"""Render simulated autonomous traces (TeamCode/build/sim/*.csv) onto a DECODE field SVG.

Usage (from the repo root, after ./gradlew :TeamCode:testDebugUnitTest):
    python TeamCode/tools/plot_sim.py [output.svg]

No third-party packages needed. Field frame = FTC SDK frame (inches, origin centre, goals at -X,
Red +Y). Drawn with the goal wall at the top and Red on the right.
"""
import csv
import math
import os
import sys

S = 5.0  # px per inch
PAD = 40
ROBOT = 18.0
TRACES = [
    ("red_goal_pgp", "#d93f3f", "Red Goal"),
    ("red_audience_pgp", "#f08a24", "Red Audience"),
    ("blue_goal_pgp", "#2f6fdb", "Blue Goal"),
    ("blue_audience_pgp", "#18a3b8", "Blue Audience"),
]


def to_px(x, y):
    return PAD + (y + 72) * S, PAD + (x + 72) * S


def poly(points, **attrs):
    pts = " ".join("%.1f,%.1f" % to_px(x, y) for x, y in points)
    a = " ".join('%s="%s"' % (k.replace("_", "-"), v) for k, v in attrs.items())
    return '<polygon points="%s" %s/>' % (pts, a)


def footprint(x, y, h_deg):
    h = math.radians(h_deg)
    out = []
    for lx, ly in ((9, 9), (9, -9), (-9, -9), (-9, 9)):
        out.append((x + lx * math.cos(h) - ly * math.sin(h), y + lx * math.sin(h) + ly * math.cos(h)))
    return out


def main():
    sim_dir = os.path.join(os.path.dirname(__file__), "..", "build", "sim")
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(sim_dir, "auto_paths.svg")
    size = 144 * S + 2 * PAD
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" font-family="sans-serif" font-size="13">' % (size, size + 90)]
    svg.append('<rect width="100%" height="100%" fill="#ffffff"/>')
    svg.append(poly([(-72, -72), (-72, 72), (72, 72), (72, -72)], fill="#eceff1", stroke="#37474f", stroke_width=3))
    for i in range(1, 6):  # tile grid
        a = -72 + 24 * i
        x1, y1 = to_px(a, -72)
        x2, y2 = to_px(a, 72)
        svg.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#cfd8dc"/>' % (x1, y1, x2, y2))
        x1, y1 = to_px(-72, a)
        x2, y2 = to_px(72, a)
        svg.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#cfd8dc"/>' % (x1, y1, x2, y2))
    svg.append(poly([(-72, 72), (-72, -72), (0, 0)], fill="#fff59d", fill_opacity=0.55, stroke="#f9a825", stroke_width=2))
    svg.append(poly([(72, 24), (72, -24), (48, 0)], fill="#fff59d", fill_opacity=0.55, stroke="#f9a825", stroke_width=2))
    for gy, col, name in ((55.6425, "#d93f3f", "RED GOAL (24)"), (-55.6425, "#2f6fdb", "BLUE GOAL (20)")):
        px, py = to_px(-58.3727, gy)
        svg.append('<circle cx="%.1f" cy="%.1f" r="7" fill="%s"/>' % (px, py, col))
        svg.append('<text x="%.1f" y="%.1f" text-anchor="middle" fill="%s" font-weight="bold">%s</text>' % (px, py + 22, col, name))
    tx, ty = to_px(-72, 0)
    svg.append('<text x="%.1f" y="%.1f" text-anchor="middle" fill="#37474f">goal wall</text>' % (tx, ty - 8))
    tx, ty = to_px(72, 0)
    svg.append('<text x="%.1f" y="%.1f" text-anchor="middle" fill="#37474f">audience wall</text>' % (tx, ty + 22))

    legend_y = size + 20
    for i, (name, col, label) in enumerate(TRACES):
        path = os.path.join(sim_dir, name + ".csv")
        if not os.path.exists(path):
            continue
        rows = list(csv.DictReader(open(path)))
        pts = [(float(r["x"]), float(r["y"])) for r in rows]
        d = " ".join(("M" if j == 0 else "L") + "%.1f,%.1f" % to_px(x, y) for j, (x, y) in enumerate(pts))
        svg.append('<path d="%s" fill="none" stroke="%s" stroke-width="3"/>' % (d, col))
        first, last = rows[0], rows[-1]
        svg.append(poly(footprint(float(first["x"]), float(first["y"]), float(first["headingDeg"])),
                        fill="none", stroke=col, stroke_width=2, stroke_dasharray="5,4"))
        svg.append(poly(footprint(float(last["x"]), float(last["y"]), float(last["headingDeg"])),
                        fill=col, fill_opacity=0.35, stroke=col, stroke_width=2))
        prev = 0
        for r in rows:
            n = int(r["shots"])
            if n > prev:
                px, py = to_px(float(r["x"]), float(r["y"]))
                svg.append('<circle cx="%.1f" cy="%.1f" r="6" fill="#ffffff" stroke="%s" stroke-width="3"/>' % (px, py, col))
                prev = n
        done = next((float(r["t"]) for r in rows if r["phase"] == "DONE"), float("nan"))
        lx = PAD + (i % 2) * 360
        svg.append('<rect x="%d" y="%d" width="14" height="14" fill="%s"/>' % (lx, legend_y - 11 + (i // 2) * 22, col))
        svg.append('<text x="%d" y="%d">%s: 3 shots, done %.1fs</text>' % (lx + 20, legend_y + (i // 2) * 22, label, done))
    svg.append('<text x="%d" y="%d" fill="#546e7a">dashed = start, filled = end of AUTO, circles = shots, yellow = launch zones</text>' % (PAD, legend_y + 50))
    svg.append("</svg>")
    with open(out_path, "w") as f:
        f.write("\n".join(svg))
    print("wrote", os.path.abspath(out_path))


if __name__ == "__main__":
    main()
