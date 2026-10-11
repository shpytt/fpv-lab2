import math
import os
import sys
import xml.etree.ElementTree as ET

x0 = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
y0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
dx = float(sys.argv[3]) if len(sys.argv) > 3 else 4.6
dy = float(sys.argv[4]) if len(sys.argv) > 4 else 1.1
tx, ty = x0 - dx, y0 - dy
CLEAR = 0.35

path = os.path.expanduser(
    "~/ardu_ws/install/ardupilot_gz_gazebo/share/"
    "ardupilot_gz_gazebo/worlds/maze.sdf"
)
root = ET.parse(path).getroot()


def pose(e):
    p = e.find("pose")
    if p is not None and p.text:
        return [float(v) for v in p.text.split()]
    return [0.0] * 6


walls = []
for link in root.iter("link"):
    for col in link.iter("collision"):
        box = col.find("geometry/box/size")
        if box is None:
            continue
        size = [float(v) for v in box.text.split()]
        lp, cp = pose(link), pose(col)
        yaw = math.radians(lp[5])
        cx = lp[0] + cp[0] * math.cos(yaw) - cp[1] * math.sin(yaw)
        cy = lp[1] + cp[0] * math.sin(yaw) + cp[1] * math.cos(yaw)
        h = size[0] / 2
        a = (cx - h * math.cos(yaw), cy - h * math.sin(yaw))
        b = (cx + h * math.cos(yaw), cy + h * math.sin(yaw))
        walls.append((link.get("name"), a, b, lp[2] + cp[2] + size[2] / 2))


def point_seg(p, a, b):
    ax, ay = a
    bx, by = b
    ex, ey = bx - ax, by - ay
    length = ex * ex + ey * ey
    t = 0.0 if length == 0 else max(
        0.0, min(1.0, ((p[0] - ax) * ex + (p[1] - ay) * ey) / length)
    )
    return math.hypot(p[0] - (ax + t * ex), p[1] - (ay + t * ey))


def ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])


def intersects(a, b, c, d):
    return ccw(a, c, d) * ccw(b, c, d) < 0 and ccw(a, b, c) * ccw(a, b, d) < 0


def seg_dist(a, b, c, d):
    if intersects(a, b, c, d):
        return 0.0
    return min(
        point_seg(a, c, d), point_seg(b, c, d),
        point_seg(c, a, b), point_seg(d, a, b),
    )


start, target = (x0, y0), (tx, ty)
print(f"Start ({x0:.2f}, {y0:.2f}) -> target ({tx:.2f}, {ty:.2f}), "
      f"distance {math.dist(start, target):.2f} m")
blocked = []
for name, a, b, top in walls:
    d = seg_dist(start, target, a, b)
    if d < CLEAR:
        blocked.append(name)
        print(f"  route crosses/touches {name}: wall {a}->{b}, top {top:.2f} m")
if not blocked:
    print("  route is clear of walls")
print("Highest wall top:", max(w[3] for w in walls))
print("Nearest wall to target: "
      f"{min(point_seg(target, a, b) for _, a, b, _ in walls):.2f} m")

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 7))
    for name, a, b, _ in walls:
        ax.plot([a[0], b[0]], [a[1], b[1]], "k-", lw=3)
        ax.text((a[0] + b[0]) / 2, (a[1] + b[1]) / 2,
                name.replace("Wall_", ""), color="r", fontsize=8)
    ax.plot(*start, "go", ms=10, label="start")
    ax.plot(*target, "bx", ms=10, label="target")
    ax.plot([x0, tx], [y0, ty], "b--")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    out = os.path.expanduser("~/lab2_report/maze_map.png")
    fig.savefig(out, dpi=120)
    print("Map saved:", out)
except ImportError:
    print("matplotlib is not installed, map was not saved")
