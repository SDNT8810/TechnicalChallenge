#!/usr/bin/env python3
"""Minimal path + waypoints plot.
Reads:
  records/decoded_blockchain_logs.json
  src/follow_waypoints_pkg/resource/odom_waypoints.csv
  src/sdnt_robot_simulation/maps/warehouse.yaml (+ image)
Outputs:
  records/path_plot.png and displays the figure.
"""
import json, os, csv, yaml, sys, numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
DECODED = ROOT / 'records' / 'decoded_blockchain_logs.json'
WAYPOINTS = ROOT / 'src' / 'follow_waypoints_pkg' / 'resource' / 'odom_waypoints.csv'
MAP_YAML = ROOT / 'src' / 'sdnt_robot_simulation' / 'maps' / 'warehouse.yaml'
OUT_PATH = ROOT / 'records' / 'path_plot.png'

def load_decoded():
    if not DECODED.exists():
        print(f"missing decoded file: {DECODED}")
        return []
    with open(DECODED) as f:
        try:
            data = json.load(f)
        except Exception as e:
            print("failed to parse decoded json", e)
            return []
    pts = []
    for row in data:
        robot = row.get('robot', {})
        x = robot.get('x'); y = robot.get('y')
        if x is not None and y is not None:
            pts.append((x, y))
    return pts

def load_waypoints():
    pts = []
    if not WAYPOINTS.exists():
        return pts
    with open(WAYPOINTS) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                pts.append((float(row['x']), float(row['y'])))
            except Exception:
                pass
    return pts

def load_map():
    if not MAP_YAML.exists():
        return None, None, None
    with open(MAP_YAML) as f:
        meta = yaml.safe_load(f)
    img_path = MAP_YAML.parent / meta['image']
    if not img_path.exists():
        return None, None, None
    import matplotlib.image as mpimg
    img = mpimg.imread(img_path)
    res = float(meta['resolution'])
    origin = meta['origin']  # [x, y, theta]
    h, w = img.shape[0], img.shape[1]
    extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
    return img, extent, meta, (w, h, res, origin)

def main():
    path_pts = load_decoded()
    wps = load_waypoints()
    map_data = load_map()
    if map_data[0] is not None:
        img, extent, meta, (w, h, res, origin) = map_data
    else:
        img, extent, meta, (w, h, res, origin) = (None, None, None, (0,0,1,[0,0,0]))

    if not path_pts and not wps:
        print("nothing to plot")
        return

    plt.figure(figsize=(6,6))

    # If map available, rotate 90 deg CW and adjust extents + coordinates
    rotated = False
    if img is not None:
        # original dimensions (w, h)
        W_m = w * res
        H_m = h * res
        # rotate image 90 deg counter-clockwise
        img_rot = np.rot90(img, 3)
        # flip horizontal
        img_rot = np.fliplr(img_rot)
        rotated = True
        # new extent after rotation (width/height swap)
        new_extent = [origin[0], origin[0] + H_m, origin[1], origin[1] + W_m]
        plt.imshow(img_rot, cmap='gray', origin='lower', extent=new_extent)
    else:
        new_extent = extent

    def rotate_points(points):
        if not rotated:
            return points
        if not points:
            return points
        W_m = w * res
        # transform each (x,y)
        out = []
        for x, y in points:
            x_rel = x - origin[0]
            y_rel = y - origin[1]
            x_r = y_rel
            y_r = (W_m - x_rel)
            out.append((origin[0] + x_r, origin[1] + y_r))
        return out

    path_pts_r = rotate_points(path_pts)
    wps_r = rotate_points(wps)

    if path_pts_r:
        xs, ys = zip(*path_pts_r)
        plt.plot(xs, ys, '-o', markersize=2, linewidth=1, color='tab:blue', label='path')
        plt.scatter(xs[0], ys[0], c='green', s=60, marker='o', label='start')
        plt.scatter(xs[-1], ys[-1], c='red', s=60, marker='X', label='end')
    if wps_r:
        wx, wy = zip(*wps_r)
        plt.scatter(wx, wy, c='orange', marker='^', s=60, label='waypoints')
        # numbered circles
        for i, (x, y) in enumerate(wps_r, start=1):
            plt.scatter([x], [y+0.3], facecolors='none', edgecolors='black', s=200)
            plt.text(x, y+0.3, str(i), ha='center', va='center', fontsize=11, color='black')

    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Robot Path & Waypoints')
    plt.legend(loc='best')
    plt.tight_layout()
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(OUT_PATH, dpi=600)
    print(f'saved {OUT_PATH}')
    plt.show()

if __name__ == '__main__':
    main()
