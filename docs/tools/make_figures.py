# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
"""Generates the documentation figures in docs/diagrams.

Usage: make_figures.py <scene_dir>
scene_dir is the output of run_synthetic_scene.sh. Needs matplotlib, scipy and graphviz (`dot`).
"""
import json
import math
import os
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle
import numpy as np
from scipy.spatial import Delaunay

SCENE_DIR = sys.argv[1]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'diagrams')

BLUE, ORANGE, GREEN, RED, GOLD, GREY, INK = '#1F5FBF', '#E08A1E', '#0F7A4E', '#B5352C', '#D9A400', '#8A96A3', '#131A21'
plt.rcParams.update({'font.size': 11, 'axes.titlesize': 12, 'figure.titlesize': 15, 'figure.titleweight': 'bold',
                     'axes.spines.top': False, 'axes.spines.right': False, 'savefig.dpi': 130,
                     'savefig.bbox': 'tight', 'savefig.facecolor': 'white'})


def save(fig, name):
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)
    print(name)


def scene(name):
    with open(os.path.join(SCENE_DIR, f'scene_{name}.json')) as f:
        return json.load(f)[name]


def dot(name, source):
    subprocess.run(['dot', '-Tpng', '-Gdpi=130', '-o', os.path.join(OUT, name)], input=source.encode(), check=True)
    print(name)


# ------------------------------------------------------------------ flow diagrams
def system_overview():
    dot('01_system_overview.png', r'''
digraph G {
  rankdir=LR; nodesep=0.3; ranksep=0.75; bgcolor=white;
  node [fontname="Helvetica", fontsize=12, shape=box, style="rounded,filled", color="#C9D0D8", fillcolor="#F3F5F8", margin="0.16,0.09"];
  edge [fontname="Helvetica", fontsize=9.5, color="#5C6975", fontcolor="#414D59"];
  client [label="Caller\n(agent, CLI, BT)", fillcolor="#FFFFFF"];
  camera [label="RGB-D camera\nimg / depth / camera_info", fillcolor="#FFFFFF"];
  subgraph cluster_repo { label="this repository"; labelloc=b; labeljust=r; fontname="Helvetica"; fontsize=12; color="#1F5FBF"; style="rounded,dashed";
    coordinator [label="reach_coordinator\n(optional proxy)\ncoordinator.py"];
    detector [label="object_detector\nDINO + SAM2, back-projection,\nrequest state, goal forwarding\ndetector.py", fillcolor="#DCE8FA"];
    surface [label="surface_detector\nheight slab, RANSAC,\nclustering, alpha shape\nsurface_detector.cpp", fillcolor="#DCF1E8"];
    planner [label="approach_planner\ncandidates, orientation gate,\ncostmap checks, selection\nplanner.cpp", fillcolor="#FBEBD0"];
  }
  costmap [label="Nav2 global costmap\n(or costmap_from_bag.launch.py)", fillcolor="#FFFFFF"];
  nav2 [label="Nav2 bt_navigator\nNavigateToPose", fillcolor="#FFFFFF"];
  tf [label="tf2\nmap > geometric_unicycle\n> realsense_compensated", fillcolor="#FFFFFF"];
  client -> coordinator [label="/reach_object_coordinator"];
  client -> detector [label="/reach_object"];
  coordinator -> detector [label="/reach_object"];
  camera -> detector;
  detector -> surface [label="/object_detector/\nsegmented_pointcloud"];
  surface -> planner [label="/surface_detector/results\nobject + contour"];
  planner -> detector [label="/approach_planner/goal_pose", color="#B5352C", fontcolor="#B5352C", constraint=false];
  detector -> nav2 [label="navigate_to_pose", color="#B5352C", fontcolor="#B5352C"];
  costmap -> planner [label="/global_costmap/costmap_raw"];
  detector -> costmap [label="/object_detector/full_pointcloud\n(costmap_from_bag.yaml only)", style=dashed];
  tf -> surface [style=dotted, arrowhead=none];
  tf -> planner [style=dotted, arrowhead=none];
}''')


def goal_forwarding():
    dot('03_goal_forwarding.png', r'''
digraph G {
  rankdir=TB; nodesep=0.3; ranksep=0.32; bgcolor=white;
  node [fontname="Helvetica", fontsize=11.5, shape=box, style="rounded,filled", color="#C9D0D8", fillcolor="#F3F5F8"];
  edge [fontname="Helvetica", fontsize=10, color="#5C6975", fontcolor="#414D59"];
  in [label="PoseStamped on /approach_planner/goal_pose", fillcolor="#DCE8FA"];
  a [label="Request active, and first frame stamped?", shape=diamond, style=filled, margin="0,0"];
  b [label="goal stamp >= first frame stamp?", shape=diamond, style=filled, margin="0,0"];
  c [label="First goal of this request?", shape=diamond, style=filled, margin="0,0"];
  d [label="Elapsed >= goal_update_min_interval?\n(default 1.0 s)", shape=diamond, style=filled, margin="0,0"];
  e [label="Moved >= 0.15 m or rotated >= 10 deg?\n(same frame as the last goal)", shape=diamond, style=filled, margin="0,0"];
  drop [label="Drop the goal\n(nothing is queued for later)", fillcolor="#F8DEDB"];
  send [label="seq += 1, reset the result status,\nsend_goal_async (preempts Nav2)", fillcolor="#DCF1E8"];
  h [label="Hands above the surface?", shape=diamond, style=filled, margin="0,0"];
  in -> a; a -> h [label=" yes"]; h -> b [label=" yes"]; h -> drop [label=" no"]; b -> c [label=" yes"]; c -> d [label=" no"]; d -> e [label=" yes"];
  c -> send [label=" yes"]; e -> send [label=" yes"];
  a -> drop [label=" no"]; b -> drop [label=" no (stale)"]; d -> drop [label=" no"]; e -> drop [label=" no"];
}''')


def planner_flow():
    dot('09_planner_filter_flow.png', r'''
digraph G {
  rankdir=TB; nodesep=0.3; ranksep=0.28; bgcolor=white;
  node [fontname="Helvetica", fontsize=11.5, shape=box, style="rounded,filled", color="#C9D0D8", fillcolor="#F3F5F8"];
  edge [fontname="Helvetica", fontsize=10, color="#5C6975", fontcolor="#414D59"];
  in [label="One candidate (x, y, theta) per contour segment", fillcolor="#DCE8FA"];
  bounds [label="Inside the costmap?", shape=diamond, style=filled, margin="0,0"];
  cost [label="cost < max_costmap_val (253)?", shape=diamond, style=filled, margin="0,0"];
  fb [label="Fallback: cheapest cell in the 5x5 window\nmoves the position, keeps theta"];
  fbok [label="Cell found?", shape=diamond, style=filled, margin="0,0"];
  clear [label="Clearance: no lethal or unknown cell\ninside the circular stencil?", shape=diamond, style=filled, margin="0,0"];
  dist [label="Distance to the object <= dist_threshold (0.6 m)?", shape=diamond, style=filled, margin="0,0"];
  local [label="Keep as a local goal", fillcolor="#FBEBD0"];
  bfs [label="One BFS from the robot cell (all local goals at once)\n4-connected, cost < 253, escape disc near the start"];
  reach [label="Cell reached?", shape=diamond, style=filled, margin="0,0"];
  hands [label="Both hands inside the contour, with margin?", shape=diamond, style=filled, margin="0,0"];
  best [label="Keep the lowest score: d_object + weight · d_robot\npublish /approach_planner/goal_pose", fillcolor="#DCF1E8"];
  drop [label="Candidate dropped", fillcolor="#F8DEDB"];
  in -> bounds; bounds -> cost [label=" yes"]; cost -> clear [label=" yes"]; cost -> fb [label=" no"];
  fb -> fbok; fbok -> clear [label=" yes"]; clear -> dist [label=" yes"]; dist -> local [label=" yes"];
  local -> bfs -> reach; reach -> hands [label=" yes"]; hands -> best [label=" yes"]; hands -> drop [label=" no"];
  bounds -> drop [label=" no"]; fbok -> drop [label=" no"]; clear -> drop [label=" no"]; dist -> drop [label=" no"]; reach -> drop [label=" no"];
}''')


def autonomy_pipeline():
    """Hand placed on a grid, so that every arrow is a straight line through empty space"""
    EXISTS, APPROACH, DESIGN, BUILD, SAFETY = '#DCF1E8', '#DCE8FA', '#FBEBD0', '#FFFFFF', '#F8DEDB'
    fig, ax = plt.subplots(figsize=(15, 12))

    def band(y0, y1, x1, title):
        ax.add_patch(FancyBboxPatch((2.0, y0), x1 - 2.0, y1 - y0, boxstyle='round,pad=0,rounding_size=0.12',
                                    fc='#F7F8FA', ec='#D5DBE1', lw=1, zorder=0))
        ax.text(0.05, (y0 + y1) / 2, title, va='center', fontsize=11, color='#414D59', fontweight='bold')

    def box(cx, cy, w, h, text, fc, ec='#AEB8C2', ls='-', lw=1.1):
        ax.add_patch(FancyBboxPatch((cx - w / 2, cy - h / 2), w, h, boxstyle='round,pad=0,rounding_size=0.1',
                                    fc=fc, ec=ec, ls=ls, lw=lw, zorder=2))
        ax.text(cx, cy, text, ha='center', va='center', fontsize=11, color=INK, zorder=3, linespacing=1.35)

    def arrow(x0, y0, x1, y1, color='#5C6975', ls='-', lw=1.5, style='-|>'):
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle=style, mutation_scale=14, color=color,
                                     ls=ls, lw=lw, shrinkA=0, shrinkB=0, zorder=1))

    def label(x, y, text, ha='left', color='#414D59'):
        ax.text(x, y, text, ha=ha, va='center', fontsize=10, color=color)

    band(9.5, 10.9, 16.6, 'Layer 1\nconversation')
    band(7.1, 8.4, 12.6, 'Layer 2\ndecisions')
    band(4.2, 6.0, 16.6, 'Layer 3\nskills')
    band(1.5, 3.1, 16.6, 'Layer 4\nrobot services')

    box(9.6, 11.95, 13.2, 0.7, 'User: voice or text', '#FFFFFF')
    box(6.8, 10.2, 5.2, 0.95, 'Chat agent (LLM)\ntalks to the user, passes on intentions', DESIGN)
    box(14.2, 10.2, 3.6, 0.95, 'Stop-all\nfixed code, no LLM', SAFETY, ec=RED)
    box(6.8, 7.75, 5.2, 0.9, 'Orchestrator agent (LLM)\ndecides how to solve the request', DESIGN)
    box(3.7, 5.2, 3.2, 1.1, 'Navigator agent\nvisual-language-navigation\nfinds objects, drives close', EXISTS)
    box(7.3, 5.2, 3.4, 1.1, 'Approach tool\nmanipulation-approach\nraises hands, closes the gap', APPROACH, ec=BLUE, lw=2.2)
    box(11.0, 5.2, 3.4, 1.1, 'Manipulation pipeline\nvision + cartesian control\ngrasp pose, grasp, IK check', BUILD, ls='--')
    box(14.8, 5.2, 2.6, 1.1, 'Head tool\nlook_at(point)', BUILD, ls='--')
    box(3.0, 2.3, 1.8, 1.0, 'SLAM +\nsemantic map\nROS 2', EXISTS)
    box(5.5, 2.3, 2.4, 1.0, 'Nav2\ncostmap, goals\nROS 2', EXISTS)
    box(9.0, 2.3, 2.6, 1.0, 'Arm controllers\nleft and right\nYARP', EXISTS)
    box(12.0, 2.3, 2.0, 1.0, 'Finger\ncontroller\nYARP', EXISTS)
    box(14.8, 2.3, 2.0, 1.0, 'Gaze\ncontroller\nYARP', EXISTS)
    box(9.6, 0.65, 13.2, 0.7, 'Robot: camera, base, arms, hands, head', '#FFFFFF')

    arrow(6.8, 11.6, 6.8, 10.68, style='<|-|>')
    arrow(14.2, 11.6, 14.2, 10.68, color=RED, lw=2)
    label(14.35, 11.14, 'stop word', color=RED)

    arrow(6.1, 9.72, 6.1, 8.2)
    label(5.95, 8.95, 'intentions, only if action is needed', ha='right')
    arrow(7.5, 8.2, 7.5, 9.72, ls='--')
    label(7.65, 8.95, 'progress, results')

    # One bus from the orchestrator to the skills
    arrow(6.8, 7.3, 6.8, 6.65, style='-')
    arrow(3.7, 6.65, 14.8, 6.65, style='-')
    for x in (3.7, 7.3, 11.0, 14.8):
        arrow(x, 6.65, x, 5.75)
    label(7.45, 6.28, '/reach_object + target hint')
    label(11.15, 6.28, 'each skill enforces its own safety')

    arrow(15.6, 9.72, 15.6, 6.0, color=RED, ls='--', lw=2)
    label(15.45, 8.0, 'cancels every skill', ha='right', color=RED)

    arrow(3.0, 4.65, 3.0, 2.8)
    arrow(4.7, 4.65, 4.7, 2.8)
    arrow(6.3, 4.65, 6.3, 2.8)
    arrow(8.2, 4.65, 8.2, 2.8)
    arrow(10.2, 4.65, 10.2, 2.8)
    arrow(12.0, 4.65, 12.0, 2.8)
    arrow(14.8, 4.65, 14.8, 2.8)
    label(5.5, 3.65, 'Nav2 goals', ha='center')
    label(9.2, 3.65, 'hand commands', ha='center')

    arrow(3.0, 1.0, 3.0, 1.8)
    label(3.15, 1.25, 'sensors')
    for x in (5.5, 9.0, 12.0, 14.8):
        arrow(x, 1.8, x, 1.0)

    # Legend, on its own row
    items = (('exists', EXISTS, '#AEB8C2', '-', 1.1), ('manipulation-approach', APPROACH, BLUE, '-', 2.2),
             ('to design', DESIGN, '#AEB8C2', '-', 1.1), ('to build', BUILD, '#AEB8C2', '--', 1.1),
             ('to build, safety', SAFETY, RED, '-', 1.1))
    for i, (text, fc, ec, ls, lw) in enumerate(items):
        box(3.9 + 2.85 * i, -0.2, 2.6, 0.45, text, fc, ec=ec, ls=ls, lw=lw)

    ax.set_xlim(0, 16.8); ax.set_ylim(-0.6, 12.5); ax.axis('off')
    fig.suptitle('Autonomy pipeline: planned architecture', y=0.93)
    save(fig, '11_autonomy_pipeline.png')


# ------------------------------------------------------------------ sequence diagram
def request_sequence():
    actors = ['Caller', 'object_detector', 'surface_detector', 'approach_planner', 'Nav2']
    xs = dict(zip(actors, [0, 3.1, 6.2, 9.3, 12.4]))
    steps = [
        ('Caller', 'object_detector', '/reach_object {object_string}', INK, None),
        ('object_detector', 'object_detector', 'goal callback: reject if busy; wait for the Nav2 server', GREY, None),
        ('object_detector', 'object_detector', 'first frame: stamp t0 is the freshness floor', GREY, None),
        ('object_detector', 'surface_detector', 'SegmentedPointcloud (stamp t)', BLUE, None),
        ('surface_detector', 'approach_planner', 'DetectionResults (stamp t)', GREEN, None),
        ('approach_planner', 'object_detector', 'goal_pose (stamp t)', ORANGE, None),
        ('object_detector', 'object_detector', 'first surface: raise both hands, then wait for the arms', GREY, None),
        ('object_detector', 'object_detector', 'keep if t >= t0, then interval + hysteresis gates', GREY, None),
        ('object_detector', 'Nav2', 'NavigateToPose (seq n)', RED, None),
        ('Nav2', 'object_detector', 'accepted: navigation has started', RED, 'dashed'),
        ('Nav2', 'object_detector', 'feedback: distance_remaining', RED, 'dashed'),
        ('object_detector', 'Caller', 'feedback: distance_remaining (5 Hz)', INK, 'dashed'),
        ('object_detector', 'Nav2', 'better goal, new frame: NavigateToPose (seq n+1), preempts seq n', RED, None),
        ('Nav2', 'object_detector', 'result of the latest seq only: SUCCEEDED', RED, 'dashed'),
        ('object_detector', 'object_detector', 'stop the search; object_string = ""', GREY, None),
        ('object_detector', 'Caller', 'result {reached: true}', INK, 'dashed'),
    ]
    fig, ax = plt.subplots(figsize=(13.5, 8.6))
    n = len(steps)
    for a in actors:
        ax.plot([xs[a]] * 2, [0.2, n + 0.6], color='#C9D0D8', lw=1.2, zorder=0)
        ax.text(xs[a], n + 0.9, a, ha='center', va='bottom', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.35', fc='#F3F5F8', ec='#C9D0D8'))
    ax.axhspan(n - 12.5, n - 2.5, xmin=0.2, color='#EEF3FB', zorder=-1)
    ax.text(12.9, n - 7, 'repeats for each\nprocessed frame', rotation=90, ha='center', va='center', color=BLUE, fontsize=10)
    for i, (src, dst, text, color, style) in enumerate(steps):
        y = n - i
        if src == dst:
            ax.text(xs[src] + 0.12, y, text, va='center', color='#414D59', style='italic', fontsize=10.5)
            ax.plot(xs[src], y, 'o', color=color, ms=4)
            continue
        ax.add_patch(FancyArrowPatch((xs[src], y), (xs[dst], y), arrowstyle='-|>', mutation_scale=13, color=color,
                                     lw=1.5, linestyle=style or 'solid'))
        ax.text((xs[src] + xs[dst]) / 2, y + 0.09, text, ha='center', va='bottom', color=color, fontsize=10.5,
                bbox=dict(fc='white', ec='none', pad=0.5, alpha=0.85))
    ax.set_xlim(-1, 13.3); ax.set_ylim(0, n + 1.5); ax.axis('off')
    fig.suptitle('Life of a /reach_object request (success path)', y=0.97)
    save(fig, '02_request_sequence.png')


# ------------------------------------------------------------------ surface detection
def draw_table(ax, tables, **kw):
    for x0, x1, y0, y1 in tables:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ls='--', ec=GREY, lw=1.1, **kw))


def surface_pipeline():
    r = scene('baseline')
    rx, ry, _ = r['scene']['robot']
    plane = np.array(r['plane'])[:, :2] + [rx, ry]
    cluster = np.array(r['cluster'])[:, :2] + [rx, ry]
    contour = np.array(r['contour_map'])
    cup = r['scene']['cup']
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.9), sharex=True, sharey=True)
    titles = [f'1. Height slab around the fitted plane\n{r["plane_n"]:,} points (both desks)',
              f'2. Cluster with the point closest to the object\n{r["cluster_n"]:,} points',
              f'3. Alpha-shape contour\n{len(contour) - 1} vertices']
    for ax, title in zip(axes, titles):
        draw_table(ax, r['scene']['tables'])
        ax.add_patch(Circle(cup, 0.04, color=RED, zorder=5))
        ax.set_title(title); ax.set_aspect('equal'); ax.set_xlabel('x [m]')
    axes[0].set_ylabel('y [m]')
    axes[0].scatter(*plane.T, s=0.6, color=BLUE)
    axes[1].scatter(*plane.T, s=0.6, color='#D5DBE1')
    axes[1].scatter(*cluster.T, s=0.6, color=GREEN)
    axes[2].scatter(*cluster.T, s=0.4, color='#D5DBE1')
    axes[2].plot(*contour.T, color=GREEN, lw=1.6)
    axes[2].annotate('interior gap behind the object:\nnarrower than 2 alpha, no hole opens', xy=(2.03, 0.33), xytext=(2.55, 1.02),
                     fontsize=9.5, arrowprops=dict(arrowstyle='->', color=INK))
    axes[0].set_xlim(1.4, 3.5); axes[0].set_ylim(-1.7, 1.25)
    fig.suptitle('surface_detector on the synthetic scene (dashed: true table outlines, red: object)', y=1.03)
    save(fig, '04_surface_pipeline.png')


def alpha_shape_boundary(points, alpha):
    """Boundary edges of the alpha complex: Delaunay triangles with circumradius <= alpha, as PCL keeps them."""
    tri = Delaunay(points)
    keep = []
    for s in tri.simplices:
        a, b, c = points[s]
        la, lb, lc = np.linalg.norm(b - c), np.linalg.norm(a - c), np.linalg.norm(a - b)
        area = abs((b - a)[0] * (c - a)[1] - (b - a)[1] * (c - a)[0]) / 2
        keep.append(area > 1e-12 and la * lb * lc / (4 * area) <= alpha)
    edges = {}
    for s, k in zip(tri.simplices, keep):
        if k:
            for e in ((s[0], s[1]), (s[1], s[2]), (s[2], s[0])):
                key = tuple(sorted(e))
                edges[key] = edges.get(key, 0) + 1
    return [points[list(e)] for e, count in edges.items() if count == 1]


def alpha_shapes():
    rng = np.random.default_rng(3)
    pts = rng.uniform([0, 0], [1.4, 0.8], (1600, 2))
    shadow = (pts[:, 0] > 0.55) & (pts[:, 0] < 0.8) & (pts[:, 1] > 0.3)     # narrow gap, as an occlusion shadow
    pts = pts[~shadow]
    fig, axes = plt.subplots(1, 3, figsize=(14, 3.9), sharex=True, sharey=True)
    for ax, alpha in zip(axes, (10.0, 0.2, 0.05)):
        ax.scatter(*pts.T, s=2.5, color='#B7C1CB')
        for e in alpha_shape_boundary(pts, alpha):
            ax.plot(*e.T, color=GREEN, lw=1.5)
        ax.add_patch(Rectangle((0.55, 0.3), 0.25, 0.5, fill=False, ls=':', ec=RED))
        ax.set_aspect('equal'); ax.set_xlabel('x [m]')
    axes[0].set_title('alpha = 10 m: behaves as a convex hull')
    axes[1].set_title('alpha = 0.2 m (this repo)\nthe 0.25 m gap is bridged, up to noise')
    axes[2].set_title('alpha = 0.05 m\nfollows the gap and the sampling noise')
    axes[0].set_ylabel('y [m]')
    fig.suptitle('Why a concave hull: effect of alpha on the same points (red: gap narrower than 2 alpha)', y=1.05)
    save(fig, '05_alpha_shape.png')


# ------------------------------------------------------------------ planner
def candidate_geometry():
    fig, ax = plt.subplots(figsize=(9.4, 6.4))
    table = np.array([[-3.2, 0], [3.6, 0], [3.6, 2.9], [-3.2, 2.9]])
    ax.add_patch(Polygon(table, fc='#EEF1F4', ec='none'))
    ax.plot([-3.2, 3.6], [0, 0], color=GREEN, lw=2.2)
    A, B, P = np.array([-1.3, 0.0]), np.array([1.5, 0.0]), np.array([0.75, 1.75])
    Q = np.array([P[0], 0.0])
    C = Q + np.array([0.0, -1.5])
    ax.plot(*zip(A, B), color=GREEN, lw=5, solid_capstyle='round')
    for name, p, off in (('A', A, (-0.08, 0.17)), ('B', B, (-0.03, 0.17))):
        ax.plot(*p, 'o', color=GREEN, ms=8); ax.text(p[0] + off[0], p[1] + off[1], name, fontweight='bold', color=GREEN)
    ax.add_patch(Circle(P, 0.14, color=RED, zorder=4))
    ax.text(P[0] + 0.2, P[1] + 0.05, 'P  object', color=RED, fontweight='bold')
    ax.plot(*Q, 's', color=INK, ms=7)
    ax.text(Q[0] - 0.14, Q[1] + 0.15, 'Q  closest point', color=INK, ha='right')
    ax.plot([P[0], Q[0]], [P[1], Q[1]], ':', color=INK)

    def arrow(a, b, color, **kw):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=15, color=color, lw=2, **kw))

    arrow(Q, C, BLUE)
    ax.text(Q[0] + 0.1, -0.78, 'outward normal\noffset = robot_radius + goal_edge_margin\n= 0.25 m',
            color=BLUE, va='center')
    G = np.array([-2.2, -1.5])
    ax.add_patch(Circle(C, 0.11, color=BLUE, zorder=4))
    ax.text(C[0] + 0.2, C[1] - 0.13, 'candidate', color=BLUE, fontweight='bold')
    ax.add_patch(Circle(G, 0.11, color=ORANGE, zorder=4))
    ax.text(-3.1, G[1] - 0.38, 'candidate of a neighbouring segment', color=ORANGE)
    u = (P - G) / np.linalg.norm(P - G)
    n = np.array([0.0, 1.0])
    d = 0.7 * u + 0.3 * n; d /= np.linalg.norm(d)
    L = 1.1
    arrow(G, G + L * u, RED)
    arrow(G, G + L * n, BLUE)
    arrow(G, G + L * d, ORANGE)
    ax.text(*(G + L * u + [0.1, -0.04]), 'to object', color=RED)
    ax.text(*(G + L * n + [-0.08, 0.02]), 'inward normal', color=BLUE, ha='right', va='center')
    ax.text(*(G + L * d + [-0.02, 0.1]), 'heading', color=ORANGE, fontweight='bold')
    ax.text(-3.0, 2.55, 'support surface', color='#5C6975')
    ax.text(2.4, -2.15, 'free space', color='#5C6975')
    ax.text(-3.1, 0.11, 'contour', color=GREEN)
    ax.set_xlim(-3.3, 3.7); ax.set_ylim(-2.3, 2.95); ax.set_aspect('equal'); ax.axis('off')
    fig.suptitle('Goal candidate geometry: one per contour segment', y=0.95)
    save(fig, '06_candidate_geometry.png')


def orientation_gate():
    wo, wn = 0.7, 0.3
    phi = np.radians(np.linspace(0, 180, 1801))
    d = np.stack([wo * np.cos(phi) + wn, wo * np.sin(phi)], 1)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    beta = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    alpha = np.degrees(phi) - beta
    limit = np.degrees(phi[np.argmax(beta > 40.0)])
    deg = np.degrees(phi)
    fig, ax = plt.subplots(figsize=(9.2, 4.7))
    ax.axvspan(0, limit, color='#DCF1E8', zorder=0)
    ax.plot(deg, alpha, color=RED, lw=2, label='heading vs. direction to the object (face check)')
    ax.plot(deg, beta, color=BLUE, lw=2, label='heading vs. inward normal (perpendicular check)')
    ax.axhline(40, color=INK, ls='--', lw=1)
    ax.text(178, 42, 'both tolerances: 40 deg', ha='right', fontsize=10)
    ax.axhline(25.4, color=RED, ls=':', lw=1)
    ax.text(178, 28.5, 'face ceiling: asin(0.3 / 0.7) = 25.4 deg, the face check cannot fail', ha='right', color=RED, fontsize=10)
    ax.axvline(limit, color=GREEN, lw=1.4)
    ax.text(limit + 2, 92, f'candidates kept up to {limit:.0f} deg', color=GREEN)
    ax.set_xlabel('angle between the inward normal and the direction to the object [deg]')
    ax.set_ylabel('angle [deg]'); ax.set_xlim(0, 180); ax.set_ylim(0, 180)
    ax.legend(loc='upper left', frameon=False, fontsize=10)
    fig.suptitle('Orientation gate with the default weights (0.7 object, 0.3 normal)', y=0.97)
    save(fig, '07_orientation_gate.png')


def costmap_law():
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw=dict(width_ratios=[1.45, 1]))
    d = np.linspace(0.0, 2.5, 800)
    for ins, color, label in ((0.1, BLUE, 'inscribed radius 0.10 m (costmap_from_bag.yaml)'),
                              (0.2, ORANGE, 'inscribed radius 0.20 m (equal to the planner robot_radius)')):
        c = 252 * np.exp(-1.5 * (d - ins))
        c[d <= ins] = 253
        ax.plot(d, c, color=color, lw=2, label=label)
    ax.axhline(253, color=RED, ls='--', lw=1)
    ax.text(2.48, 240, 'max_costmap_val = 253', color=RED, ha='right', fontsize=10)
    ax.axvline(0.25, color=GREEN, lw=1.3)
    ax.text(0.27, 25, 'goal offset 0.25 m\ncost 201 / 233', color=GREEN, fontsize=10)
    ax.set_xlabel('distance to the nearest lethal cell [m]'); ax.set_ylabel('cost')
    ax.set_title('Nav2 inflation law, cost_scaling_factor = 1.5'); ax.set_xlim(0, 2.5); ax.set_ylim(0, 265)
    ax.legend(frameon=False, fontsize=9.5, loc='upper center', bbox_to_anchor=(0.55, 0.9))

    for x in range(-5, 6):
        for y in range(-5, 6):
            color = '#DCF1E8' if x * x + y * y <= 16 else '#F3F5F8'
            bx.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, fc=color, ec='white'))
    bx.add_patch(Circle((0, 0), 4, fill=False, ec=INK, ls='--'))
    bx.plot(0, 0, 'o', color=INK)
    bx.set_title('Clearance stencil: robot_radius 0.2 m at 5 cm per cell\n49 cells checked')
    bx.set_xlim(-5.6, 5.6); bx.set_ylim(-6.3, 5.6); bx.set_aspect('equal'); bx.axis('off')
    fig.suptitle('The costmap checks', y=1.08)
    save(fig, '08_costmap_checks.png')


def worked_examples():
    panels = [('baseline', 'Baseline: free near edge'), ('blocked', 'Near edge blocked in the costmap'),
              ('fov_clipped', 'Too close: near edge out of view'), ('too_far', 'Object 0.7 m from the nearest edge')]
    cmap = ListedColormap(['#FFFFFF', '#FDF5E6', '#F8E2BB', '#E8A98A', '#3B4650'])
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 10.2))
    for ax, (name, title) in zip(axes.flat, panels):
        r = scene(name)
        cost = np.array(r['costmap'], dtype=float)
        o, res, size = r['grid']['origin'], r['grid']['res'], r['grid']['size']
        bins = np.digitize(cost, [1, 120, 200, 254])
        ax.imshow(bins, cmap=cmap, vmin=0, vmax=4, origin='lower', interpolation='nearest',
                  extent=[o[0], o[0] + size * res, o[1], o[1] + size * res])
        contour = np.array(r['contour_map'])
        ax.plot(*contour.T, color=GREEN, lw=1.5, label='contour')
        if r.get('cand'):
            ax.scatter(*np.array(r['cand']).T, s=11, color=BLUE, label='candidates', zorder=3)
        if r.get('filt'):
            ax.scatter(*np.array(r['filt']).T, s=24, color=GOLD, ec=INK, lw=0.4, label='filtered', zorder=4)
        ax.add_patch(Circle(r['scene']['cup'], 0.05, color=RED, zorder=6))
        x, y, yaw = r['scene']['robot']
        ax.add_patch(Circle((x, y), 0.2, fc='#DCE8FA', ec=BLUE, zorder=5))
        ax.text(x, y - 0.33, 'robot', ha='center', color=BLUE, fontsize=10, zorder=8)
        if r.get('goal'):
            gx, gy, gyaw = r['goal'][:3]
            ax.add_patch(FancyArrowPatch((gx, gy), (gx + 0.42 * math.cos(gyaw), gy + 0.42 * math.sin(gyaw)),
                                         arrowstyle='-|>', mutation_scale=16, color=RED, lw=2.4, zorder=7))
            ax.set_title(f'{title}\ngoal ({gx:.2f}, {gy:.2f}), yaw {math.degrees(gyaw):.0f} deg, '
                         f'{len(r["filt"])} of {len(r["cand"])} candidates kept')
        else:
            ax.set_title(f'{title}\nno goal, 0 of {len(r["cand"])} candidates kept')
        ax.set_xlim(0.2, 3.3); ax.set_ylim(-1.7, 1.5); ax.set_aspect('equal')
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    axes[0, 0].legend(loc='lower left', frameon=True, fontsize=9.5)
    fig.suptitle('Real approach_planner output on the synthetic scene (red dot: object, red arrow: goal)', y=0.97)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    save(fig, '10_worked_examples.png')


system_overview()
autonomy_pipeline()
request_sequence()
goal_forwarding()
surface_pipeline()
alpha_shapes()
candidate_geometry()
orientation_gate()
costmap_law()
planner_flow()
worked_examples()
