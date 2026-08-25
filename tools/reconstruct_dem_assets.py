#!/usr/bin/env python3
"""Reconstruct Gazebo DEM assets from the retained processed elevation grid."""

import argparse
import struct
from pathlib import Path

import numpy as np
from PIL import Image


def write_binary_stl(path, elevation, origin_x, origin_y, resolution):
    height, width = elevation.shape
    triangles = (height - 1) * (width - 1) * 2
    with path.open("wb") as stream:
        stream.write(b"reconstructed from terrain_layers_hard.npz".ljust(80, b" "))
        stream.write(struct.pack("<I", triangles))
        for row in range(height - 1):
            for col in range(width - 1):
                p00 = (origin_x + col * resolution, origin_y + row * resolution,
                       float(elevation[row, col]))
                p10 = (origin_x + (col + 1) * resolution, origin_y + row * resolution,
                       float(elevation[row, col + 1]))
                p01 = (origin_x + col * resolution, origin_y + (row + 1) * resolution,
                       float(elevation[row + 1, col]))
                p11 = (origin_x + (col + 1) * resolution, origin_y + (row + 1) * resolution,
                       float(elevation[row + 1, col + 1]))
                for a, b, c in ((p00, p10, p11), (p00, p11, p01)):
                    ux, uy, uz = (b[i] - a[i] for i in range(3))
                    vx, vy, vz = (c[i] - a[i] for i in range(3))
                    normal = (uy * vz - uz * vy, uz * vx - ux * vz,
                              ux * vy - uy * vx)
                    stream.write(struct.pack("<3f", *normal))
                    stream.write(struct.pack("<9f", *(a + b + c)))
                    stream.write(struct.pack("<H", 0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--npz", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--origin-x", type=float, default=-9.5238095)
    parser.add_argument("--origin-y", type=float, default=-15.0)
    parser.add_argument("--resolution", type=float, default=0.1)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    with np.load(args.npz) as data:
        elevation = np.asarray(data["elevation_m"], dtype=np.float32)
    if elevation.ndim != 2 or not np.isfinite(elevation).all():
        raise ValueError("elevation_m must be a finite 2-D array")
    low, high = float(elevation.min()), float(elevation.max())
    scale = max(high - low, 1.0e-9)
    image = np.round((elevation - low) / scale * 65535.0).astype(np.uint16)
    png = args.output_dir / "dem_terrain_hard_husky_015_513.png"
    image_path = args.output_dir / "dem_uniform_gray_surface.png"
    # Gazebo classic requires a square heightmap; the retained cost grid is
    # 190x300, so resample it to the 513x513 size used by the original asset.
    resized = Image.fromarray(image.astype(np.float32), mode="F").resize(
        (513, 513), resample=Image.BILINEAR)
    # Gazebo Classic 11 may render a 16-bit PNG while silently failing to
    # construct its ODE heightmap collision. Use an 8-bit grayscale PNG for
    # portable visual/collision loading; the resulting vertical resolution is
    # about 6 mm for this 1.514 m terrain range.
    heightmap = Image.fromarray(
        np.clip(np.rint(np.asarray(resized) / 257.0), 0, 255).astype(np.uint8),
        mode="L")
    heightmap.save(png)
    Image.new("L", (513, 513), 180).save(image_path)
    stl = args.output_dir / "dem_terrain_hard_husky_visual.stl"
    write_binary_stl(stl, elevation, args.origin_x, args.origin_y, args.resolution)
    print({"elevation_shape": list(elevation.shape), "min_m": low, "max_m": high,
           "heightmap": str(png), "mesh": str(stl), "surface": str(image_path)})


if __name__ == "__main__":
    main()
