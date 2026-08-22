#!/usr/bin/env python3
"""Endpoint-only Husky / Gazebo Classic penetration diagnostic.

Run while Gazebo is active and Husky has already stopped at the target.
The program does not change the world, robot, controller, or DEM.
"""

import argparse
import json
import math
import os
import re
import statistics
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

try:
    from PIL import Image
except Exception:
    Image = None

try:
    import cv2
except Exception:
    cv2 = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState
from rcl_interfaces.srv import GetParameters


WHEEL_KEYS = {
    "front_left": ("front_left_wheel", "front_left"),
    "front_right": ("front_right_wheel", "front_right"),
    "rear_left": ("rear_left_wheel", "rear_left"),
    "rear_right": ("rear_right_wheel", "rear_right"),
}
CHASSIS_TOKENS = (
    "base_link", "base_collision", "chassis", "top_plate",
    "front_bumper", "rear_bumper", "battery_mount", "user_rail",
)
TERRAIN_TOKENS = ("terrain", "heightmap", "dem", "ground", "world")


def local_name(tag):
    return tag.split("}")[-1]


def child_text(elem, name, default=None):
    for child in list(elem):
        if local_name(child.tag) == name:
            return (child.text or "").strip()
    return default


def parse_floats(text, n=None, default=None):
    try:
        vals = [float(v) for v in str(text).replace(",", " ").split()]
        if n is not None and len(vals) < n:
            vals.extend([0.0] * (n - len(vals)))
        return vals if n is None else vals[:n]
    except Exception:
        return default


def quat_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def pose_matrix(values):
    vals = list(values) + [0.0] * max(0, 6 - len(values))
    x, y, z, r, p, yy = vals[:6]
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(r, p, yy)
    T[:3, 3] = [x, y, z]
    return T


def quat_pose_matrix(pose):
    q = pose.orientation
    r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(r, p, y)
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def transform_point(T, point):
    p = np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return (T @ p)[:3]


def percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), q))


def safe_mean(values):
    return float(np.mean(values)) if values else None


def safe_median(values):
    return float(np.median(values)) if values else None


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose_to_dict(pose):
    q = pose.orientation
    r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
    return {
        "x": float(pose.position.x), "y": float(pose.position.y), "z": float(pose.position.z),
        "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
        "roll_rad": r, "pitch_rad": p, "yaw_rad": y,
        "roll_deg": math.degrees(r), "pitch_deg": math.degrees(p), "yaw_deg": math.degrees(y),
    }


def load_grayscale(path):
    if Image is not None:
        img = Image.open(path)
        arr = np.asarray(img)
    elif cv2 is not None:
        arr = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if arr is None:
            raise RuntimeError("OpenCV could not read the heightmap")
    else:
        raise RuntimeError("Neither Pillow nor OpenCV is available to read the PNG heightmap")

    if arr.ndim == 3:
        arr = arr[..., :3].astype(np.float64)
        arr = 0.299 * arr[..., 0] + 0.587 * arr[..., 1] + 0.114 * arr[..., 2]
    else:
        arr = arr.astype(np.float64)

    if np.issubdtype(arr.dtype, np.floating) and np.nanmax(arr) <= 1.0:
        max_value = 1.0
    else:
        original = Image.open(path) if Image is not None else None
        if original is not None and getattr(original, "mode", "") in ("I;16", "I;16B", "I;16L"):
            max_value = 65535.0
        else:
            observed = float(np.nanmax(arr))
            max_value = 65535.0 if observed > 255.0 else 255.0
    return arr, max_value


def resolve_uri(uri, world_path):
    if not uri:
        return None
    uri = uri.strip()
    if uri.startswith("file://"):
        p = Path(uri[7:]).expanduser()
        return str(p.resolve()) if p.exists() else str(p)
    if uri.startswith("model://"):
        rel = uri[len("model://"):]
        search = []
        search.extend([Path(p) for p in os.environ.get("GAZEBO_MODEL_PATH", "").split(":") if p])
        search.extend([Path.home() / ".gazebo/models", Path("/usr/share/gazebo-11/models")])
        for root in search:
            p = root / rel
            if p.exists():
                return str(p.resolve())
        return uri
    if uri.startswith("package://"):
        rel = uri[len("package://"):]
        parts = rel.split("/", 1)
        if len(parts) == 2:
            package, rest = parts
            candidates = [
                Path.home() / "terrain_radiation_ws/src" / package / rest,
                Path.home() / "terrain_radiation_ws/install" / package / "share" / package / rest,
                Path("/opt/ros/foxy/share") / package / rest,
            ]
            for p in candidates:
                if p.exists():
                    return str(p.resolve())
        return uri
    p = Path(uri).expanduser()
    if not p.is_absolute():
        p = Path(world_path).parent / p
    return str(p.resolve()) if p.exists() else str(p)


def parse_heightmaps(world_path):
    root = ET.parse(world_path).getroot()
    records = []
    warnings = []

    def walk(elem, T_parent, context_kind=None, context_name=None):
        tag = local_name(elem.tag)
        T_here = T_parent
        if tag in ("world", "model", "link", "collision", "visual"):
            pose_el = next((c for c in list(elem) if local_name(c.tag) == "pose"), None)
            if pose_el is not None:
                if pose_el.attrib.get("relative_to"):
                    warnings.append("A pose uses relative_to; simple world parser assumes normal parent-relative frames.")
                vals = parse_floats(pose_el.text or "", 6, [0.0] * 6)
                T_here = T_parent @ pose_matrix(vals)
        if tag in ("collision", "visual"):
            context_kind = tag
            context_name = elem.attrib.get("name", "")
        if tag == "heightmap":
            uri = child_text(elem, "uri", "")
            size = parse_floats(child_text(elem, "size", ""), 3, None)
            pos = parse_floats(child_text(elem, "pos", "0 0 0"), 3, [0.0, 0.0, 0.0])
            sampling_text = child_text(elem, "sampling", "1")
            try:
                sampling = int(float(sampling_text))
            except Exception:
                sampling = 1
            T_hm = T_here @ pose_matrix([pos[0], pos[1], pos[2], 0, 0, 0])
            R = T_hm[:3, :3]
            terrain_rpy = matrix_to_rpy(R)
            records.append({
                "kind": context_kind or "unknown",
                "name": context_name or "",
                "uri": uri,
                "resolved_uri": resolve_uri(uri, world_path),
                "size": size,
                "pos_world": [float(v) for v in T_hm[:3, 3]],
                "rotation_rpy": [float(v) for v in terrain_rpy],
                "sampling": sampling,
            })
        for child in list(elem):
            if local_name(child.tag) == "pose":
                continue
            walk(child, T_here, context_kind, context_name)

    walk(root, np.eye(4))
    return records, warnings


def matrix_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        roll = math.atan2(-float(R[1, 2]), float(R[1, 1]))
        yaw = 0.0
    return roll, pitch, yaw


class HeightmapSampler:
    def __init__(self, config, fallback_path=None):
        self.config = dict(config)
        path = self.config.get("resolved_uri")
        if (not path or not Path(path).exists()) and fallback_path:
            path = str(Path(fallback_path).expanduser())
            self.config["resolved_uri"] = path
        if not path or not Path(path).exists():
            raise FileNotFoundError("Heightmap image not found: {}".format(path))
        if not self.config.get("size"):
            raise RuntimeError("Heightmap <size> is missing")
        self.path = path
        self.arr, self.pixel_max = load_grayscale(path)
        self.h, self.w = self.arr.shape[:2]
        self.sx, self.sy, self.sz = [float(v) for v in self.config["size"]]
        self.px, self.py, self.pz = [float(v) for v in self.config.get("pos_world", [0, 0, 0])]
        rot = self.config.get("rotation_rpy", [0, 0, 0])
        self.rotation_supported = max(abs(float(v)) for v in rot) < 1e-6

    def _pixel(self, x, y, orientation):
        fx = (x - (self.px - self.sx / 2.0)) / self.sx
        fy = (y - (self.py - self.sy / 2.0)) / self.sy
        u = fx * (self.w - 1)
        if orientation == "row0_ymin":
            v = fy * (self.h - 1)
        else:
            v = (1.0 - fy) * (self.h - 1)
        return u, v

    def sample(self, x, y, orientation):
        if not self.rotation_supported:
            return None
        u, v = self._pixel(x, y, orientation)
        if u < 0 or v < 0 or u > self.w - 1 or v > self.h - 1:
            return None
        u0, v0 = int(math.floor(u)), int(math.floor(v))
        u1, v1 = min(u0 + 1, self.w - 1), min(v0 + 1, self.h - 1)
        du, dv = u - u0, v - v0
        val = (
            self.arr[v0, u0] * (1 - du) * (1 - dv)
            + self.arr[v0, u1] * du * (1 - dv)
            + self.arr[v1, u0] * (1 - du) * dv
            + self.arr[v1, u1] * du * dv
        )
        return self.pz + (float(val) / self.pixel_max) * self.sz


class GzContactCollector:
    def __init__(self, preferred_topic):
        self.preferred_topic = preferred_topic
        self.topic = None
        self.proc = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.depths = defaultdict(list)
        self.positions = defaultdict(list)
        self.blocks = defaultdict(int)
        self.collision_names = set()
        self.raw_lines_seen = 0
        self.stderr_tail = []

    def discover_topic(self):
        try:
            out = subprocess.run(["gz", "topic", "-l"], text=True, capture_output=True, timeout=5)
            topics = [line.strip() for line in out.stdout.splitlines() if line.strip()]
        except Exception:
            topics = []
        if self.preferred_topic in topics:
            return self.preferred_topic
        candidates = [t for t in topics if "contact" in t.lower()]
        exactish = [t for t in candidates if t.endswith("/physics/contacts")]
        return (exactish or candidates or [self.preferred_topic])[0]

    def start(self):
        self.topic = self.discover_topic()
        try:
            help_text = subprocess.run(["gz", "topic", "--help"], text=True, capture_output=True, timeout=3).stdout
        except Exception:
            help_text = ""
        if "--topic" in help_text or "-t" in help_text:
            cmd = ["gz", "topic", "-e", "-t", self.topic]
        else:
            cmd = ["gz", "topic", "-e", self.topic]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return cmd

    def stop(self):
        self.stop_event.set()
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.thread:
            self.thread.join(timeout=2)
        if self.proc and self.proc.stderr:
            try:
                tail = self.proc.stderr.read().splitlines()[-20:]
                self.stderr_tail.extend(tail)
            except Exception:
                pass

    def _append_capped(self, target, value, cap=250000):
        if len(target) < cap:
            target.append(value)

    def _classify(self, c1, c2):
        s = (c1 + " " + c2).lower()
        for key, tokens in WHEEL_KEYS.items():
            if any(tok in s for tok in tokens):
                return key
        # Any robot non-wheel geometry touching the terrain is treated as chassis/body contact.
        if any(tok in s for tok in CHASSIS_TOKENS):
            return "chassis"
        return "other"

    def _finalize(self, contact):
        c1 = contact.get("collision1", "")
        c2 = contact.get("collision2", "")
        category = self._classify(c1, c2)
        with self.lock:
            self.blocks[category] += 1
            self.collision_names.update([c1, c2])
            for d in contact.get("depths", []):
                self._append_capped(self.depths[category], float(d))
            for p in contact.get("positions", []):
                self._append_capped(self.positions[category], tuple(float(v) for v in p), cap=50000)

    def _run(self):
        if not self.proc or not self.proc.stdout:
            return
        in_contact = False
        brace_depth = 0
        contact = None
        in_position = False
        position_depth = 0
        pos = {}
        for raw in self.proc.stdout:
            if self.stop_event.is_set():
                break
            self.raw_lines_seen += 1
            line = raw.strip()
            if not in_contact:
                if re.match(r"^contact\s*\{", line):
                    in_contact = True
                    brace_depth = line.count("{") - line.count("}")
                    contact = {"collision1": "", "collision2": "", "depths": [], "positions": []}
                continue

            if in_position:
                m = re.match(r"^(x|y|z):\s*([-+0-9.eE]+)", line)
                if m:
                    pos[m.group(1)] = float(m.group(2))
                position_depth += line.count("{") - line.count("}")
                if position_depth <= 0:
                    if all(k in pos for k in ("x", "y", "z")):
                        contact["positions"].append((pos["x"], pos["y"], pos["z"]))
                    in_position = False
                    pos = {}
                brace_depth += line.count("{") - line.count("}")
                if brace_depth <= 0:
                    self._finalize(contact)
                    in_contact = False
                    contact = None
                continue

            m = re.match(r'^collision1:\s*"(.*)"', line)
            if m:
                contact["collision1"] = m.group(1)
            m = re.match(r'^collision2:\s*"(.*)"', line)
            if m:
                contact["collision2"] = m.group(1)
            m = re.match(r"^depth:\s*([-+0-9.eE]+)", line)
            if m:
                contact["depths"].append(float(m.group(1)))
            if re.match(r"^position\s*\{", line):
                in_position = True
                position_depth = line.count("{") - line.count("}")
                pos = {}

            brace_depth += line.count("{") - line.count("}")
            if brace_depth <= 0:
                self._finalize(contact)
                in_contact = False
                contact = None

    def snapshot(self):
        with self.lock:
            result = {
                "topic": self.topic,
                "raw_lines_seen": self.raw_lines_seen,
                "stderr_tail": list(self.stderr_tail),
                "collision_names": sorted(n for n in self.collision_names if n),
                "categories": {},
            }
            for key in list(WHEEL_KEYS.keys()) + ["chassis", "other"]:
                ds = list(self.depths.get(key, []))
                ps = list(self.positions.get(key, []))
                result["categories"][key] = {
                    "contact_blocks": int(self.blocks.get(key, 0)),
                    "depth_count": len(ds),
                    "depth_median_m": safe_median(ds),
                    "depth_p95_m": percentile(ds, 95),
                    "depth_p99_m": percentile(ds, 99),
                    "depth_max_m": max(ds) if ds else None,
                    "positions": ps,
                }
            return result


class DiagnosticNode(Node):
    def __init__(self, odom_topic, cmd_topics):
        super().__init__("husky_endpoint_penetration_diagnostic")
        qos = QoSProfile(depth=200)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.durability = QoSDurabilityPolicy.VOLATILE
        self.odom_samples = []
        self.cmd_samples = defaultdict(list)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)
        self.cmd_subs = []
        for topic in cmd_topics:
            self.cmd_subs.append(self.create_subscription(Twist, topic, lambda msg, t=topic: self._cmd_cb(msg, t), qos))

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.odom_samples.append({
            "wall_time": time.time(),
            "ros_stamp_sec": int(msg.header.stamp.sec),
            "ros_stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": float(pose.position.x), "y": float(pose.position.y), "z": float(pose.position.z),
            "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
            "roll_rad": r, "pitch_rad": p, "yaw_rad": y,
            "linear_x": float(msg.twist.twist.linear.x),
            "linear_y": float(msg.twist.twist.linear.y),
            "linear_z": float(msg.twist.twist.linear.z),
            "angular_x": float(msg.twist.twist.angular.x),
            "angular_y": float(msg.twist.twist.angular.y),
            "angular_z": float(msg.twist.twist.angular.z),
        })
        if len(self.odom_samples) > 10000:
            self.odom_samples = self.odom_samples[-10000:]

    def _cmd_cb(self, msg, topic):
        self.cmd_samples[topic].append({
            "wall_time": time.time(),
            "linear_x": float(msg.linear.x), "linear_y": float(msg.linear.y), "linear_z": float(msg.linear.z),
            "angular_x": float(msg.angular.x), "angular_y": float(msg.angular.y), "angular_z": float(msg.angular.z),
        })
        if len(self.cmd_samples[topic]) > 10000:
            self.cmd_samples[topic] = self.cmd_samples[topic][-10000:]

    def fetch_robot_description(self, timeout_per_service=1.0):
        services = self.get_service_names_and_types()
        candidates = []
        for name, types in services:
            if name.endswith("/get_parameters") and any("rcl_interfaces/srv/GetParameters" in t for t in types):
                candidates.append(name)
        candidates.sort(key=lambda n: ("robot_state_publisher" not in n, n))
        for service_name in candidates:
            client = self.create_client(GetParameters, service_name)
            if not client.wait_for_service(timeout_sec=0.2):
                continue
            req = GetParameters.Request()
            req.names = ["robot_description"]
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_per_service)
            if fut.done() and fut.result() and fut.result().values:
                val = fut.result().values[0]
                text = getattr(val, "string_value", "")
                if "<robot" in text and "<link" in text:
                    return text, service_name
        return None, None

    def discover_entity_service(self):
        services = self.get_service_names_and_types()
        candidates = []
        for name, types in services:
            if any("gazebo_msgs/srv/GetEntityState" in t for t in types):
                candidates.append(name)
        candidates.sort(key=lambda n: (n != "/get_entity_state", n))
        return candidates[0] if candidates else None

    def get_entity_state(self, service_name, entity_name, timeout=2.0):
        if not service_name:
            return None
        client = self.create_client(GetEntityState, service_name)
        if not client.wait_for_service(timeout_sec=0.5):
            return None
        req = GetEntityState.Request()
        req.name = entity_name
        req.reference_frame = "world"
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return None
        resp = fut.result()
        if not bool(resp.success):
            return {"success": False, "entity": entity_name}
        return {
            "success": True,
            "entity": entity_name,
            "pose": pose_to_dict(resp.state.pose),
            "twist": {
                "linear_x": float(resp.state.twist.linear.x),
                "linear_y": float(resp.state.twist.linear.y),
                "linear_z": float(resp.state.twist.linear.z),
                "angular_x": float(resp.state.twist.angular.x),
                "angular_y": float(resp.state.twist.angular.y),
                "angular_z": float(resp.state.twist.angular.z),
            },
            "reference_frame": str(resp.state.reference_frame),
        }


def infer_model_names(collision_names, fallback):
    names = []
    for c in collision_names:
        parts = c.split("::")
        if len(parts) >= 2:
            head = parts[0]
            low = c.lower()
            if any(k in low for k in tuple(sum((list(v) for v in WHEEL_KEYS.values()), [])) + CHASSIS_TOKENS):
                names.append(head)
    if fallback:
        names.append(fallback)
    seen = []
    for n in names:
        if n and n not in seen:
            seen.append(n)
    return seen or ["husky"]


def parse_urdf_geometries(robot_description, link_name="base_link"):
    root = ET.fromstring(robot_description)
    links = [e for e in root.iter() if local_name(e.tag) == "link"]
    link = next((e for e in links if e.attrib.get("name") == link_name), None)
    if link is None:
        link = next((e for e in links if e.attrib.get("name", "").endswith("base_link")), None)
    if link is None:
        return [], {"error": "base_link not found in robot_description"}
    geoms = []
    unsupported = []
    for collision in [c for c in list(link) if local_name(c.tag) == "collision"]:
        origin = next((c for c in list(collision) if local_name(c.tag) == "origin"), None)
        xyz = parse_floats(origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0", 3, [0, 0, 0])
        rpy = parse_floats(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0", 3, [0, 0, 0])
        geom = next((c for c in list(collision) if local_name(c.tag) == "geometry"), None)
        if geom is None:
            continue
        shape = next(iter(list(geom)), None)
        if shape is None:
            continue
        tag = local_name(shape.tag)
        rec = {"name": collision.attrib.get("name", ""), "type": tag, "xyz": xyz, "rpy": rpy}
        if tag == "box":
            rec["size"] = parse_floats(shape.attrib.get("size", ""), 3, None)
            if rec["size"]:
                geoms.append(rec)
        elif tag == "cylinder":
            try:
                rec["radius"] = float(shape.attrib["radius"])
                rec["length"] = float(shape.attrib["length"])
                geoms.append(rec)
            except Exception:
                unsupported.append(rec)
        elif tag == "sphere":
            try:
                rec["radius"] = float(shape.attrib["radius"])
                geoms.append(rec)
            except Exception:
                unsupported.append(rec)
        else:
            rec["filename"] = shape.attrib.get("filename") or shape.attrib.get("url")
            rec["scale"] = parse_floats(shape.attrib.get("scale", "1 1 1"), 3, [1, 1, 1])
            unsupported.append(rec)
    return geoms, {"link_name": link.attrib.get("name"), "unsupported": unsupported}


def geometry_surface_points(geom):
    points = []
    typ = geom["type"]
    if typ == "box":
        sx, sy, sz = geom["size"]
        xs = np.linspace(-sx / 2, sx / 2, 7)
        ys = np.linspace(-sy / 2, sy / 2, 7)
        zs = np.linspace(-sz / 2, sz / 2, 5)
        for x in xs:
            for y in ys:
                points.append((x, y, -sz / 2))
                points.append((x, y, sz / 2))
        for x in xs:
            for z in zs:
                points.append((x, -sy / 2, z))
                points.append((x, sy / 2, z))
        for y in ys:
            for z in zs:
                points.append((-sx / 2, y, z))
                points.append((sx / 2, y, z))
    elif typ == "cylinder":
        r, length = geom["radius"], geom["length"]
        for z in (-length / 2, length / 2):
            for rr in np.linspace(0, r, 5):
                for theta in np.linspace(0, 2 * math.pi, 32, endpoint=False):
                    points.append((rr * math.cos(theta), rr * math.sin(theta), z))
        for z in np.linspace(-length / 2, length / 2, 9):
            for theta in np.linspace(0, 2 * math.pi, 32, endpoint=False):
                points.append((r * math.cos(theta), r * math.sin(theta), z))
    elif typ == "sphere":
        r = geom["radius"]
        for phi in np.linspace(0, math.pi, 17):
            for theta in np.linspace(0, 2 * math.pi, 32, endpoint=False):
                points.append((r * math.sin(phi) * math.cos(theta), r * math.sin(phi) * math.sin(theta), r * math.cos(phi)))
    return points


def calculate_base_clearance(geometries, base_pose_dict, sampler, orientation):
    if not geometries or not base_pose_dict or sampler is None:
        return None
    T_base = pose_matrix([
        base_pose_dict["x"], base_pose_dict["y"], base_pose_dict["z"],
        base_pose_dict["roll_rad"], base_pose_dict["pitch_rad"], base_pose_dict["yaw_rad"],
    ])
    all_results = []
    for geom in geometries:
        T_collision = T_base @ pose_matrix(geom["xyz"] + geom["rpy"])
        for point in geometry_surface_points(geom):
            world = transform_point(T_collision, point)
            terrain_z = sampler.sample(float(world[0]), float(world[1]), orientation)
            if terrain_z is not None:
                all_results.append({
                    "clearance_m": float(world[2] - terrain_z),
                    "point_world": [float(v) for v in world],
                    "terrain_z_m": float(terrain_z),
                    "collision_name": geom.get("name", ""),
                    "geometry_type": geom["type"],
                })
    if not all_results:
        return None
    worst = min(all_results, key=lambda r: r["clearance_m"])
    return {
        "minimum_clearance_m": worst["clearance_m"],
        "base_link_lowest_sample_z_m": min(r["point_world"][2] for r in all_results),
        "worst_sample": worst,
        "sample_count": len(all_results),
    }


def compare_heightmaps(a, b):
    if not a or not b:
        return {"match": None, "differences": ["visual or collision heightmap missing"]}
    diffs = []
    if os.path.realpath(str(a.get("resolved_uri"))) != os.path.realpath(str(b.get("resolved_uri"))):
        diffs.append("URI/image differs")
    for field in ("size", "pos_world", "rotation_rpy"):
        av, bv = a.get(field), b.get(field)
        if av is None or bv is None or len(av) != len(bv) or max(abs(float(x) - float(y)) for x, y in zip(av, bv)) > 1e-7:
            diffs.append(field + " differs")
    if int(a.get("sampling", 1)) != int(b.get("sampling", 1)):
        diffs.append("sampling differs")
    return {"match": not diffs, "differences": diffs}


def calibrate_orientation(sampler, contact_snapshot):
    positions = []
    for wheel in WHEEL_KEYS:
        positions.extend(contact_snapshot["categories"].get(wheel, {}).get("positions", []))
    if len(positions) > 5000:
        step = max(1, len(positions) // 5000)
        positions = positions[::step][:5000]
    results = {}
    for orientation in ("row0_ymin", "row0_ymax"):
        residuals = []
        signed = []
        for x, y, z in positions:
            pred = sampler.sample(x, y, orientation)
            if pred is not None:
                residuals.append(abs(float(z) - pred))
                signed.append(float(z) - pred)
        results[orientation] = {
            "sample_count": len(residuals),
            "median_abs_error_m": safe_median(residuals),
            "p95_abs_error_m": percentile(residuals, 95),
            "median_signed_error_m": safe_median(signed),
        }
    valid = [(k, v) for k, v in results.items() if v["sample_count"] >= 10 and v["median_abs_error_m"] is not None]
    if valid:
        chosen, stats = min(valid, key=lambda kv: kv[1]["median_abs_error_m"])
        reliable = stats["sample_count"] >= 50 and stats["median_abs_error_m"] <= 0.03
    else:
        chosen, reliable = "row0_ymax", False
    return {"chosen": chosen, "reliable": reliable, "orientations": results}


def summarize_odom(samples, goal_x, goal_y):
    if not samples:
        return {"sample_count": 0}
    first, last = samples[0], samples[-1]
    zs = [s["z"] for s in samples]
    rolls = [s["roll_rad"] for s in samples]
    pitches = [s["pitch_rad"] for s in samples]
    linear_speed = [math.sqrt(s["linear_x"] ** 2 + s["linear_y"] ** 2 + s["linear_z"] ** 2) for s in samples]
    angular_speed = [math.sqrt(s["angular_x"] ** 2 + s["angular_y"] ** 2 + s["angular_z"] ** 2) for s in samples]
    return {
        "sample_count": len(samples),
        "first": first,
        "last": last,
        "duration_wall_s": float(last["wall_time"] - first["wall_time"]),
        "goal_error_m": math.hypot(last["x"] - goal_x, last["y"] - goal_y),
        "xy_drift_first_to_last_m": math.hypot(last["x"] - first["x"], last["y"] - first["y"]),
        "z_change_first_to_last_m": float(last["z"] - first["z"]),
        "z_range_m": float(max(zs) - min(zs)),
        "roll_range_deg": math.degrees(max(rolls) - min(rolls)),
        "pitch_range_deg": math.degrees(max(pitches) - min(pitches)),
        "max_odom_linear_speed_m_s": max(linear_speed),
        "max_odom_angular_speed_rad_s": max(angular_speed),
    }


def summarize_cmd(cmd_samples, start_time, ignore_first_s=2.0):
    result = {}
    for topic, samples in cmd_samples.items():
        filtered = [s for s in samples if s["wall_time"] >= start_time + ignore_first_s]
        def mag(s):
            lin = math.sqrt(s["linear_x"] ** 2 + s["linear_y"] ** 2 + s["linear_z"] ** 2)
            ang = math.sqrt(s["angular_x"] ** 2 + s["angular_y"] ** 2 + s["angular_z"] ** 2)
            return lin, ang
        mags = [mag(s) for s in filtered]
        result[topic] = {
            "sample_count": len(samples),
            "post_grace_sample_count": len(filtered),
            "last": samples[-1] if samples else None,
            "max_linear_m_s_after_grace": max((m[0] for m in mags), default=None),
            "max_angular_rad_s_after_grace": max((m[1] for m in mags), default=None),
            "nonzero_after_grace": any(m[0] > 1e-3 or m[1] > 1e-3 for m in mags),
        }
    return result


def gz_model_pose(model_name):
    try:
        p = subprocess.run(["gz", "model", "-m", model_name, "-p"], text=True, capture_output=True, timeout=5)
        vals = parse_floats(p.stdout.strip(), 6, None)
        if p.returncode == 0 and vals and len(vals) == 6:
            return {
                "success": True,
                "entity": model_name,
                "source": "gz model -p",
                "pose": {
                    "x": vals[0], "y": vals[1], "z": vals[2],
                    "roll_rad": vals[3], "pitch_rad": vals[4], "yaw_rad": vals[5],
                    "roll_deg": math.degrees(vals[3]), "pitch_deg": math.degrees(vals[4]), "yaw_deg": math.degrees(vals[5]),
                },
            }
        return {"success": False, "entity": model_name, "stderr": p.stderr.strip(), "stdout": p.stdout.strip()}
    except Exception as exc:
        return {"success": False, "entity": model_name, "error": str(exc)}


def make_verdict(result):
    physical = []
    visual = []
    control = []
    missing = []

    contacts = result.get("contacts", {}).get("categories", {})
    chassis = contacts.get("chassis", {})
    if chassis.get("contact_blocks", 0) > 0:
        max_depth = chassis.get("depth_max_m")
        if max_depth is not None and max_depth > 0.003:
            physical.append("chassis contacted terrain with depth > 3 mm")
        else:
            physical.append("chassis-terrain contact was detected")

    clearance = result.get("base_link_clearance")
    calibration = result.get("heightmap_orientation_calibration", {})
    if clearance and calibration.get("reliable"):
        c = clearance.get("minimum_clearance_m")
        if c is not None and c < -0.01:
            physical.append("base_link collision surface is > 10 mm below calibrated collision terrain")
        elif c is not None and c >= -0.005:
            visual.append("base_link collision clearance is non-negative within 5 mm tolerance")
    elif clearance is None:
        missing.append("base_link clearance unavailable")
    else:
        missing.append("heightmap orientation/contact calibration not reliable enough for clearance verdict")

    odom = result.get("ground_truth_odom", {})
    if odom.get("sample_count", 0) == 0:
        missing.append("no ground-truth odometry received")
    else:
        if odom.get("z_change_first_to_last_m", 0.0) < -0.02 or odom.get("z_range_m", 0.0) > 0.03:
            physical.append("robot z changed abnormally during the stopped observation window")
        if odom.get("roll_range_deg", 0.0) > 3.0 or odom.get("pitch_range_deg", 0.0) > 3.0:
            physical.append("roll/pitch continued changing by more than 3 degrees after stopping")

    cmd = result.get("cmd_vel", {})
    for topic, stats in cmd.items():
        if stats.get("nonzero_after_grace"):
            control.append("{} still published non-zero velocity after the 2 s grace period".format(topic))

    comparison = result.get("visual_collision_comparison", {})
    if comparison.get("match") is False:
        visual.append("visual and collision heightmap configurations differ: " + ", ".join(comparison.get("differences", [])))

    wheel_ok = True
    wheel_seen = True
    for key in WHEEL_KEYS:
        cat = contacts.get(key, {})
        p95 = cat.get("depth_p95_m")
        if cat.get("contact_blocks", 0) <= 0:
            wheel_seen = False
        if p95 is None or p95 >= 0.003:
            wheel_ok = False
    if wheel_seen and wheel_ok:
        visual.append("all four wheels contacted terrain and every wheel P95 depth is below 3 mm")
    if chassis.get("contact_blocks", 0) == 0 and result.get("contacts", {}).get("raw_lines_seen", 0) > 0:
        visual.append("no chassis-terrain contact was detected")

    stable = (
        odom.get("sample_count", 0) > 0
        and odom.get("z_range_m", 999) <= 0.02
        and odom.get("roll_range_deg", 999) <= 2.0
        and odom.get("pitch_range_deg", 999) <= 2.0
    )
    no_control = not control
    no_physical = not physical
    enough_physics = result.get("contacts", {}).get("raw_lines_seen", 0) > 0 and odom.get("sample_count", 0) > 0

    if physical:
        verdict = "REAL_PHYSICAL_PENETRATION_OR_SETTLING"
        zh = "检测到真实物理穿透或停车后继续下沉/倾斜的证据。"
    elif control:
        verdict = "POST_STOP_COMMAND_RESIDUAL"
        zh = "没有直接确认车体穿透，但停车后仍存在非零速度命令，应先处理控制残余。"
    elif enough_physics and stable and no_control and chassis.get("contact_blocks", 0) == 0 and (wheel_ok or (clearance and calibration.get("reliable") and clearance.get("minimum_clearance_m", -1) >= -0.005)):
        verdict = "PHYSICS_NORMAL_VISUAL_OR_CAMERA_RENDERING_ISSUE"
        zh = "物理状态正常；截图中的“陷入”更可能是 terrain visual、相机裁剪/遮挡或机器人 visual 渲染问题。"
    elif comparison.get("match") is False and no_physical:
        verdict = "VISUAL_COLLISION_HEIGHTMAP_MISMATCH"
        zh = "未发现明确物理穿透，但 visual/collision heightmap 配置不一致。"
    else:
        verdict = "INCONCLUSIVE_MISSING_OR_CONFLICTING_EVIDENCE"
        zh = "证据不足或互相矛盾，暂时不能可靠区分真实穿透与视觉问题。"

    return {
        "code": verdict,
        "conclusion_zh": zh,
        "physical_evidence": physical,
        "visual_or_normal_physics_evidence": visual,
        "control_evidence": control,
        "missing_or_unreliable": missing,
    }


def format_mm(v):
    return "N/A" if v is None else "{:.3f} mm".format(v * 1000.0)


def write_report(path, data):
    lines = []
    lines.append("HUSKY ENDPOINT PENETRATION DIAGNOSTIC")
    lines.append("Generated: {}".format(data["generated_at"]))
    lines.append("=" * 72)
    v = data["verdict"]
    lines.append("VERDICT: {}".format(v["code"]))
    lines.append(v["conclusion_zh"])
    lines.append("")

    odom = data.get("ground_truth_odom", {})
    lines.append("[1] Ground-truth odometry")
    lines.append("samples: {}".format(odom.get("sample_count", 0)))
    if odom.get("last"):
        last = odom["last"]
        lines.append("final xyz: ({:.6f}, {:.6f}, {:.6f}) m".format(last["x"], last["y"], last["z"]))
        lines.append("final rpy: ({:.3f}, {:.3f}, {:.3f}) deg".format(math.degrees(last["roll_rad"]), math.degrees(last["pitch_rad"]), math.degrees(last["yaw_rad"])))
        lines.append("goal error: {:.4f} m".format(odom.get("goal_error_m", float("nan"))))
        lines.append("z range: {:.3f} mm; z change: {:.3f} mm".format(1000 * odom.get("z_range_m", 0), 1000 * odom.get("z_change_first_to_last_m", 0)))
        lines.append("roll range: {:.3f} deg; pitch range: {:.3f} deg".format(odom.get("roll_range_deg", 0), odom.get("pitch_range_deg", 0)))
    lines.append("")

    lines.append("[2] Gazebo entity state")
    lines.append(json.dumps(data.get("gazebo_entity_states", {}), ensure_ascii=False, indent=2))
    lines.append("")

    lines.append("[3] Heightmap visual/collision")
    lines.append(json.dumps(data.get("visual_collision_comparison", {}), ensure_ascii=False, indent=2))
    lines.append("orientation calibration: " + json.dumps(data.get("heightmap_orientation_calibration", {}), ensure_ascii=False, indent=2))
    lines.append("terrain heights: " + json.dumps(data.get("terrain_heights", {}), ensure_ascii=False, indent=2))
    lines.append("")

    lines.append("[4] base_link collision clearance")
    lines.append(json.dumps(data.get("base_link_clearance", {}), ensure_ascii=False, indent=2))
    lines.append("")

    lines.append("[5] Wheel and chassis contacts")
    cats = data.get("contacts", {}).get("categories", {})
    for key in list(WHEEL_KEYS.keys()) + ["chassis", "other"]:
        c = cats.get(key, {})
        lines.append("{}: blocks={}, median={}, P95={}, P99={}, max={}".format(
            key, c.get("contact_blocks", 0), format_mm(c.get("depth_median_m")),
            format_mm(c.get("depth_p95_m")), format_mm(c.get("depth_p99_m")),
            format_mm(c.get("depth_max_m"))))
    lines.append("")

    lines.append("[6] cmd_vel after stop")
    lines.append(json.dumps(data.get("cmd_vel", {}), ensure_ascii=False, indent=2))
    lines.append("")

    for title, items in (
        ("Physical evidence", v["physical_evidence"]),
        ("Normal physics / visual evidence", v["visual_or_normal_physics_evidence"]),
        ("Control evidence", v["control_evidence"]),
        ("Missing/unreliable", v["missing_or_unreliable"]),
    ):
        lines.append(title + ":")
        if items:
            lines.extend("- " + item for item in items)
        else:
            lines.append("- none")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--goal-x", type=float, default=-1.13)
    parser.add_argument("--goal-y", type=float, default=-7.80)
    parser.add_argument("--world", default=str(Path.home() / "terrain_radiation_ws/src/radiation_mapping/worlds/module36_hard_radiation_plugin.world"))
    parser.add_argument("--heightmap", default=str(Path.home() / "terrain_radiation_ws/src/radiation_mapping/dem/processed/dem_terrain_hard_husky_015_513.png"))
    parser.add_argument("--odom-topic", default="/ground_truth/odom")
    parser.add_argument("--contact-topic", default="/gazebo/dem_inspired_benchmark_world/physics/contacts")
    parser.add_argument("--model-name", default="husky")
    parser.add_argument("--base-link", default="base_link")
    parser.add_argument("--output-root", default=str(Path.home() / "terrain_radiation_ws/diagnostics"))
    args = parser.parse_args()

    output_dir = Path(args.output_root).expanduser() / ("endpoint_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    output_dir.mkdir(parents=True, exist_ok=True)

    print("[diagnostic] Parsing world and heightmap configuration...")
    heightmap_records, world_warnings = parse_heightmaps(args.world)
    collision_hms = [r for r in heightmap_records if r["kind"] == "collision"]
    visual_hms = [r for r in heightmap_records if r["kind"] == "visual"]
    collision_hm = collision_hms[0] if collision_hms else None
    visual_hm = visual_hms[0] if visual_hms else None
    visual_collision_comparison = compare_heightmaps(collision_hm, visual_hm)

    sampler = None
    sampler_error = None
    if collision_hm:
        try:
            sampler = HeightmapSampler(collision_hm, args.heightmap)
        except Exception as exc:
            sampler_error = str(exc)

    rclpy.init()
    node = DiagnosticNode(args.odom_topic, ["/cmd_vel", "/husky_velocity_controller/cmd_vel_unstamped"])

    # Give graph discovery a short moment.
    discovery_end = time.time() + 1.0
    while time.time() < discovery_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("[diagnostic] Fetching robot_description and Gazebo service information...")
    robot_description, robot_description_service = node.fetch_robot_description()
    entity_service = node.discover_entity_service()

    collector = GzContactCollector(args.contact_topic)
    contact_cmd = collector.start()
    print("[diagnostic] Contact stream: {}".format(collector.topic))
    print("[diagnostic] Observing stopped robot for {:.1f} s...".format(args.duration))

    start_time = time.time()
    end_time = start_time + args.duration
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.05)
    collector.stop()
    contact_snapshot = collector.snapshot()

    odom_summary = summarize_odom(node.odom_samples, args.goal_x, args.goal_y)
    cmd_summary = summarize_cmd(node.cmd_samples, start_time)

    model_names = infer_model_names(contact_snapshot.get("collision_names", []), args.model_name)
    entity_states = {"service": entity_service, "queries": {}}
    selected_base_pose = odom_summary.get("last")
    selected_base_source = "ground_truth_odom"
    if entity_service:
        for model in model_names:
            candidates = [model + "::" + args.base_link, model]
            for entity in candidates:
                state = node.get_entity_state(entity_service, entity)
                entity_states["queries"][entity] = state
                if state and state.get("success") and entity.endswith("::" + args.base_link):
                    selected_base_pose = state["pose"]
                    selected_base_source = "GetEntityState:" + entity
                    break
            if selected_base_source.startswith("GetEntityState"):
                break
    if not entity_service:
        for model in model_names:
            state = gz_model_pose(model)
            entity_states["queries"][model] = state

    calibration = None
    terrain_heights = {"sampler_error": sampler_error}
    if sampler:
        calibration = calibrate_orientation(sampler, contact_snapshot)
        orientation = calibration["chosen"]
        if odom_summary.get("last"):
            last = odom_summary["last"]
            terrain_heights["collision_at_final_xy_m"] = sampler.sample(last["x"], last["y"], orientation)
        terrain_heights["collision_at_goal_xy_m"] = sampler.sample(args.goal_x, args.goal_y, orientation)
        terrain_heights["orientation"] = orientation
        terrain_heights["image_path"] = sampler.path
        terrain_heights["image_width"] = sampler.w
        terrain_heights["image_height"] = sampler.h
        terrain_heights["pixel_max"] = sampler.pixel_max
    else:
        calibration = {"chosen": None, "reliable": False, "orientations": {}}

    base_geometries = []
    urdf_info = {"robot_description_service": robot_description_service}
    if robot_description:
        try:
            base_geometries, parsed_info = parse_urdf_geometries(robot_description, args.base_link)
            urdf_info.update(parsed_info)
            urdf_info["supported_collision_geometries"] = base_geometries
        except Exception as exc:
            urdf_info["error"] = str(exc)
    else:
        urdf_info["error"] = "robot_description could not be retrieved"

    base_clearance = None
    if sampler and calibration.get("chosen") and selected_base_pose and base_geometries:
        base_clearance = calculate_base_clearance(base_geometries, selected_base_pose, sampler, calibration["chosen"])
        if base_clearance:
            base_clearance["base_pose_source"] = selected_base_source

    # Remove raw position arrays from compact JSON after calibration; retain statistics only.
    for cat in contact_snapshot.get("categories", {}).values():
        cat["position_count"] = len(cat.get("positions", []))
        cat.pop("positions", None)

    data = {
        "generated_at": datetime.now().isoformat(),
        "arguments": vars(args),
        "world_heightmaps": heightmap_records,
        "world_parser_warnings": world_warnings,
        "visual_collision_comparison": visual_collision_comparison,
        "heightmap_orientation_calibration": calibration,
        "terrain_heights": terrain_heights,
        "ground_truth_odom": odom_summary,
        "gazebo_entity_states": entity_states,
        "selected_base_pose_source": selected_base_source,
        "robot_description": urdf_info,
        "base_link_clearance": base_clearance,
        "contacts": contact_snapshot,
        "contact_command": contact_cmd,
        "cmd_vel": cmd_summary,
    }
    data["verdict"] = make_verdict(data)

    json_path = output_dir / "diagnostic.json"
    report_path = output_dir / "report.txt"
    json_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    write_report(report_path, data)

    print("\n" + "=" * 72)
    print("VERDICT: " + data["verdict"]["code"])
    print(data["verdict"]["conclusion_zh"])
    print("=" * 72)
    if odom_summary.get("last"):
        last = odom_summary["last"]
        print("Final ground truth: x={:.4f}, y={:.4f}, z={:.4f} m".format(last["x"], last["y"], last["z"]))
        print("RPY: roll={:.3f}, pitch={:.3f}, yaw={:.3f} deg".format(
            math.degrees(last["roll_rad"]), math.degrees(last["pitch_rad"]), math.degrees(last["yaw_rad"])))
        print("Goal error: {:.4f} m".format(odom_summary.get("goal_error_m", float("nan"))))
        print("Stopped-window z range: {:.3f} mm".format(1000 * odom_summary.get("z_range_m", 0)))
    if base_clearance:
        print("Minimum base_link collision clearance: {:.3f} mm".format(1000 * base_clearance["minimum_clearance_m"]))
        print("Clearance source: {} | heightmap calibration reliable={}".format(selected_base_source, calibration.get("reliable")))
    else:
        print("Minimum base_link collision clearance: unavailable (see report)")
    for key in list(WHEEL_KEYS.keys()) + ["chassis"]:
        c = contact_snapshot["categories"].get(key, {})
        print("{}: blocks={}, P95={}, max={}".format(key, c.get("contact_blocks", 0), format_mm(c.get("depth_p95_m")), format_mm(c.get("depth_max_m"))))
    for topic, stats in cmd_summary.items():
        print("{}: messages={}, nonzero_after_grace={}".format(topic, stats["sample_count"], stats["nonzero_after_grace"]))
    print("Report: {}".format(report_path))
    print("JSON:   {}".format(json_path))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
