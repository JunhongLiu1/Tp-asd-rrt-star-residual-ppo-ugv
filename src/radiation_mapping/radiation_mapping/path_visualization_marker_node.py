import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class PathVisualizationMarkerNode(Node):
    def __init__(self):
        super().__init__('path_visualization_marker_node')

        self.publisher = self.create_publisher(
            MarkerArray,
            'candidate_paths_marker',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_markers)

        self.start_point = (-4.0, 0.0)
        self.goal_point = (4.0, 0.0)

        self.paths = [
            {
                'name': 'Path A: radiation-risk',
                'points': [
                    (-4.0, 0.0),
                    (-2.0, 1.0),
                    (0.0, 1.5),
                    (2.0, 2.5),
                    (4.0, 0.0),
                ],
                'color': (1.0, 0.0, 0.0, 1.0),
                'id': 0,
            },
            {
                'name': 'Path B: terrain-risk',
                'points': [
                    (-4.0, 0.0),
                    (-2.0, -0.5),
                    (0.0, -1.0),
                    (2.0, -1.5),
                    (4.0, 0.0),
                ],
                'color': (1.0, 0.6, 0.0, 1.0),
                'id': 10,
            },
            {
                'name': 'Path C: safe detour',
                'points': [
                    (-4.0, 0.0),
                    (-4.0, -3.0),
                    (0.0, -4.5),
                    (4.0, -2.0),
                    (4.0, 0.0),
                ],
                'color': (0.0, 1.0, 0.0, 1.0),
                'id': 20,
            },
        ]

        self.get_logger().info('Same start-goal path visualization node started.')

    def create_line_marker(self, path):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'candidate_paths'
        marker.id = path['id']
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.08

        r, g, b, a = path['color']
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        marker.pose.orientation.w = 1.0

        for x, y in path['points']:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.15
            marker.points.append(p)

        return marker

    def create_text_marker(self, path):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'candidate_path_labels'
        marker.id = path['id'] + 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.text = path['name']

        last_x, last_y = path['points'][-1]
        marker.pose.position.x = last_x
        marker.pose.position.y = last_y
        marker.pose.position.z = 0.6
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.3

        r, g, b, a = path['color']
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker

    def create_point_marker(self, point, marker_id, text, color):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'start_goal_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        return marker

    def create_label_marker(self, point, marker_id, text, color):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'start_goal_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.text = text

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.7
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.35

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        return marker

    def publish_markers(self):
        marker_array = MarkerArray()

        for path in self.paths:
            marker_array.markers.append(self.create_line_marker(path))
            marker_array.markers.append(self.create_text_marker(path))

        marker_array.markers.append(
            self.create_point_marker(
                self.start_point,
                100,
                'Start',
                (0.0, 0.0, 1.0, 1.0)
            )
        )

        marker_array.markers.append(
            self.create_label_marker(
                self.start_point,
                101,
                'Start',
                (0.0, 0.0, 1.0, 1.0)
            )
        )

        marker_array.markers.append(
            self.create_point_marker(
                self.goal_point,
                102,
                'Goal',
                (1.0, 0.0, 1.0, 1.0)
            )
        )

        marker_array.markers.append(
            self.create_label_marker(
                self.goal_point,
                103,
                'Goal',
                (1.0, 0.0, 1.0, 1.0)
            )
        )

        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizationMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
