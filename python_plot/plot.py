import csv
import os
import heapq
import matplotlib.pyplot as plt

# =========================
# Match C++ map settings
# =========================
W = 30
H = 20

occ = [[0 for _ in range(W)] for _ in range(H)]

# Static obstacles (same as main.cpp)
for yy in range(6, 15):
    occ[yy][12] = 1
for yy in range(2, 9):
    occ[yy][20] = 1
for xx in range(6, 11):
    occ[10][xx] = 1

start_cell = (2, 2)
goal_cell = (26, 16)

def cell_center(cx, cy):
    return (cx + 0.5, cy + 0.5)

# =========================
# Risk cost map (same as C++)
# =========================
import math
def risk_cost(cx, cy):
    dx1 = cx - 10
    dy1 = cy - 4
    r1 = math.sqrt(dx1*dx1 + dy1*dy1)

    dx2 = cx - 22
    dy2 = cy - 14
    r2 = math.sqrt(dx2*dx2 + dy2*dy2)

    cost = 0.0
    if r1 < 6.0:
        cost += (6.0 - r1) * 0.35
    if r2 < 7.0:
        cost += (7.0 - r2) * 0.25
    return cost

cost_map = [[risk_cost(x, y) for x in range(W)] for y in range(H)]

# =========================
# A* (Python rebuild)
# =========================
def in_bounds(cx, cy):
    return 0 <= cx < W and 0 <= cy < H

def heuristic(ax, ay, gx, gy):
    return abs(gx - ax) + abs(gy - ay)

def astar(occ_grid, start, goal):
    sx, sy = start
    gx, gy = goal

    gScore = [[10**9 for _ in range(W)] for _ in range(H)]
    parent = [[None for _ in range(W)] for _ in range(H)]
    visited = [[False for _ in range(W)] for _ in range(H)]

    gScore[sy][sx] = 0.0
    pq = []
    heapq.heappush(pq, (heuristic(sx, sy, gx, gy), 0.0, sx, sy))

    dirs = [(1,0), (-1,0), (0,1), (0,-1)]

    while pq:
        f, g, cx, cy = heapq.heappop(pq)
        if visited[cy][cx]:
            continue
        visited[cy][cx] = True

        if (cx, cy) == (gx, gy):
            break

        for dx, dy in dirs:
            nx, ny = cx + dx, cy + dy
            if not in_bounds(nx, ny):
                continue
            if occ_grid[ny][nx] == 1:
                continue

            move_cost = 1.0 + cost_map[ny][nx]
            cand = gScore[cy][cx] + move_cost

            if cand < gScore[ny][nx]:
                gScore[ny][nx] = cand
                parent[ny][nx] = (cx, cy)
                nf = cand + heuristic(nx, ny, gx, gy)
                heapq.heappush(pq, (nf, cand, nx, ny))

    if parent[gy][gx] is None and (sx, sy) != (gx, gy):
        return []

    path_cells = []
    cur = (gx, gy)
    path_cells.append(cur)

    while cur != (sx, sy):
        cur = parent[cur[1]][cur[0]]
        if cur is None:
            return []
        path_cells.append(cur)

    path_cells.reverse()
    return path_cells

path_cells = astar(occ, start_cell, goal_cell)
path_x = [cell_center(cx, cy)[0] for cx, cy in path_cells]
path_y = [cell_center(cx, cy)[1] for cx, cy in path_cells]

# =========================
# Load CSV log
# =========================
path = "../logs/auv_log.csv"

t, state = [], []
depth_true, depth_meas, depth_est = [], [], []
target, error, u = [], [], []
x, y = [], []
battery, event = [], []
replan_count = []

with open(path, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row["t"]))
        state.append(row["mission_state"])
        depth_true.append(float(row["depth_true"]))
        depth_meas.append(float(row["depth_meas"]))
        depth_est.append(float(row["depth_est"]))
        target.append(float(row["target_depth"]))
        error.append(float(row["depth_error"]))
        u.append(float(row["u"]))
        x.append(float(row["x"]))
        y.append(float(row["y"]))
        battery.append(float(row["battery"]))
        event.append(row["event"])
        replan_count.append(int(row["replan_count"]))

# state transitions
transition_times = []
prev = state[0]
for i in range(1, len(state)):
    if state[i] != prev:
        transition_times.append((t[i], state[i]))
        prev = state[i]

# replans
replan_times = []
prev_r = replan_count[0]
for i in range(1, len(replan_count)):
    if replan_count[i] != prev_r:
        replan_times.append(t[i])
        prev_r = replan_count[i]

# =========================
# Plot
# =========================
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
(ax1, ax2), (ax3, ax4) = axes

# Depth
ax1.plot(t, depth_true, label="depth_true")
ax1.plot(t, depth_meas, label="depth_meas", linestyle=":")
ax1.plot(t, depth_est, label="depth_est (Kalman)")
ax1.plot(t, target, label="target_depth")
ax1.set_title("Depth (True vs Measured vs Kalman Estimate)")
ax1.set_xlabel("time (s)")
ax1.set_ylabel("depth (m)")
ax1.legend()
for ts, _ in transition_times:
    ax1.axvline(ts, linestyle="--")

# Error
ax2.plot(t, error, label="depth_error")
ax2.set_title("Depth Error (using Kalman estimate)")
ax2.set_xlabel("time (s)")
ax2.set_ylabel("error (m)")
ax2.legend()
for ts, _ in transition_times:
    ax2.axvline(ts, linestyle="--")

# Control + battery
step = 10
ax3.plot(t[::step], u[::step], label="thrust u")
ax3.set_title("Control Effort + Battery")
ax3.set_xlabel("time (s)")
ax3.set_ylabel("u")
ax3.legend(loc="upper left")
ax3b = ax3.twinx()
ax3b.plot(t, battery, label="battery", linestyle=":")
ax3b.set_ylabel("battery (%)")
lines1, labels1 = ax3.get_legend_handles_labels()
lines2, labels2 = ax3b.get_legend_handles_labels()
ax3b.legend(lines1 + lines2, labels1 + labels2, loc="upper right")
for ts, _ in transition_times:
    ax3.axvline(ts, linestyle="--")

# Map + obstacles + costmap + path + trajectory
ax4.set_title("2D Map: Cost Zones + Obstacles + A* Path + Trajectory + Replans")
ax4.set_xlabel("x (m)")
ax4.set_ylabel("y (m)")
ax4.axis("equal")

# cost map background
# imshow expects [row=y][col=x]
ax4.imshow(cost_map, origin="lower", extent=[0, W, 0, H], alpha=0.35)

# obstacles
ox, oy = [], []
for yy in range(H):
    for xx in range(W):
        if occ[yy][xx] == 1:
            ox.append(xx + 0.5)
            oy.append(yy + 0.5)
ax4.scatter(ox, oy, marker="s", label="obstacles")

# A* path
if len(path_x) > 0:
    ax4.plot(path_x, path_y, linestyle="--", label="A* planned path")

# trajectory
ax4.plot(x, y, label="trajectory")

# start/goal
sx, sy = cell_center(*start_cell)
gx, gy = cell_center(*goal_cell)
ax4.scatter([sx], [sy], label="start")
ax4.scatter([gx], [gy], label="goal")

# replans
for rt_i, rt in enumerate(replan_times):
    idx = min(range(len(t)), key=lambda i: abs(t[i] - rt))
    ax4.scatter([x[idx]], [y[idx]], marker="o", label="replan" if rt_i == 0 else None)

ax4.legend()

plt.tight_layout()

# ✅ Save plot for README
os.makedirs("../plots", exist_ok=True)
plt.savefig("../plots/report.png", dpi=200)

plt.show()
