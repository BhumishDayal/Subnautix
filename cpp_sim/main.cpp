#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <queue>
#include <random>

using namespace std;

// ---------------- Helpers ----------------
static double clamp(double x, double lo, double hi) { return max(lo, min(hi, x)); }

struct PID {
    double kp, ki, kd;
    double integral = 0.0;
    double prev_error = 0.0;

    double step(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void reset() { integral = 0.0; prev_error = 0.0; }
};

// ---------------- 1D Kalman Filter (Depth + Velocity) ----------------
struct KalmanDepth {
    double depth = 0.0;
    double vel = 0.0;

    double P00 = 1.0, P01 = 0.0;
    double P10 = 0.0, P11 = 1.0;

    double Q_depth = 0.02;
    double Q_vel   = 0.05;
    double R_meas  = 0.25;

    void predict(double dt) {
        double d_pred = depth + dt * vel;
        double v_pred = vel;

        double P00n = P00 + dt*(P10 + P01) + dt*dt*P11 + Q_depth;
        double P01n = P01 + dt*P11;
        double P10n = P10 + dt*P11;
        double P11n = P11 + Q_vel;

        depth = d_pred;
        vel   = v_pred;

        P00 = P00n; P01 = P01n;
        P10 = P10n; P11 = P11n;
    }

    void update(double z_measured) {
        double y = z_measured - depth;
        double S = P00 + R_meas;
        double K0 = P00 / S;
        double K1 = P10 / S;

        depth += K0 * y;
        vel   += K1 * y;

        double P00n = (1 - K0) * P00;
        double P01n = (1 - K0) * P01;
        double P10n = P10 - K1 * P00;
        double P11n = P11 - K1 * P01;

        P00 = P00n; P01 = P01n;
        P10 = P10n; P11 = P11n;
    }
};

// ---------------- AUV State ----------------
struct AUVState {
    double t = 0.0;

    double depth_true = 0.0;
    double depth_vel  = 0.0;

    double x = 0.0;
    double y = 0.0;

    double vx = 0.0;
    double vy = 0.0;

    double battery = 100.0;
};

// ---------------- Physics ----------------
void physics_step(AUVState& s, double u, double dt) {
    const double thrust_gain = 2.0;
    const double drag = 1.0;
    const double disturbance = 0.03 * sin(3.0 * s.t) + 0.02 * cos(7.0 * s.t);

    double acc = thrust_gain * u - drag * s.depth_vel + disturbance;
    s.depth_vel += acc * dt;
    s.depth_true += s.depth_vel * dt;

    if (s.depth_true < 0.0) {
        s.depth_true = 0.0;
        if (s.depth_vel < 0.0) s.depth_vel = 0.0;
    }

    s.x += s.vx * dt;
    s.y += s.vy * dt;

    double base = 0.018;
    double vertical = 0.08 * abs(u);
    double planar = 0.02 * sqrt(s.vx*s.vx + s.vy*s.vy);

    s.battery -= (base + vertical + planar) * dt;
    if (s.battery < 0.0) s.battery = 0.0;

    s.t += dt;
}

// ---------------- Mission States ----------------
enum class MissionState {
    ARM,
    DIVE,
    NAVIGATE,
    REPLAN,
    SURFACE,
    EMERGENCY_SURFACE,
    DONE
};

static string state_to_string(MissionState s) {
    switch (s) {
        case MissionState::ARM: return "ARM";
        case MissionState::DIVE: return "DIVE";
        case MissionState::NAVIGATE: return "NAVIGATE";
        case MissionState::REPLAN: return "REPLAN";
        case MissionState::SURFACE: return "SURFACE";
        case MissionState::EMERGENCY_SURFACE: return "EMERGENCY_SURFACE";
        case MissionState::DONE: return "DONE";
    }
    return "UNKNOWN";
}

// ---------------- Grid + A* ----------------
struct Cell { int x, y; };

struct Node {
    int x, y;
    double f, g;
};

struct NodeCmp {
    bool operator()(const Node& a, const Node& b) const {
        return a.f > b.f;
    }
};

static bool in_bounds(int x, int y, int W, int H) {
    return x >= 0 && x < W && y >= 0 && y < H;
}

static double heuristic(int x, int y, int gx, int gy) {
    return abs(gx - x) + abs(gy - y);
}

// ✅ COST MAP / RISK ZONES (currents, terrain risk)
// returns extra cost for entering a cell
static double risk_cost(int cx, int cy) {
    // Two "risk zones" (high current / unsafe terrain)
    // Zone 1: around (10, 4)
    double dx1 = cx - 10;
    double dy1 = cy - 4;
    double r1 = sqrt(dx1*dx1 + dy1*dy1);

    // Zone 2: around (22, 14)
    double dx2 = cx - 22;
    double dy2 = cy - 14;
    double r2 = sqrt(dx2*dx2 + dy2*dy2);

    double cost = 0.0;
    if (r1 < 6.0) cost += (6.0 - r1) * 0.35;
    if (r2 < 7.0) cost += (7.0 - r2) * 0.25;

    return cost;
}

static vector<Cell> astar_path(const vector<vector<int>>& occ, Cell start, Cell goal) {
    int H = (int)occ.size();
    int W = (int)occ[0].size();

    vector<vector<double>> gScore(H, vector<double>(W, 1e18));
    vector<vector<pair<int,int>>> parent(H, vector<pair<int,int>>(W, {-1,-1}));
    vector<vector<int>> visited(H, vector<int>(W, 0));

    priority_queue<Node, vector<Node>, NodeCmp> pq;

    gScore[start.y][start.x] = 0.0;
    pq.push({start.x, start.y, heuristic(start.x, start.y, goal.x, goal.y), 0.0});

    int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

    while (!pq.empty()) {
        Node cur = pq.top(); pq.pop();

        if (visited[cur.y][cur.x]) continue;
        visited[cur.y][cur.x] = 1;

        if (cur.x == goal.x && cur.y == goal.y) break;

        for (auto& d : dirs) {
            int nx = cur.x + d[0];
            int ny = cur.y + d[1];

            if (!in_bounds(nx, ny, W, H)) continue;
            if (occ[ny][nx] == 1) continue;

            // ✅ weighted cost: base move cost + risk
            double move_cost = 1.0 + risk_cost(nx, ny);
            double cand = gScore[cur.y][cur.x] + move_cost;

            if (cand < gScore[ny][nx]) {
                gScore[ny][nx] = cand;
                parent[ny][nx] = {cur.x, cur.y};
                double f = cand + heuristic(nx, ny, goal.x, goal.y);
                pq.push({nx, ny, f, cand});
            }
        }
    }

    vector<Cell> path;
    if (parent[goal.y][goal.x].first == -1 && !(goal.x == start.x && goal.y == start.y)) {
        return {};
    }

    int cx = goal.x, cy = goal.y;
    path.push_back({cx, cy});
    while (!(cx == start.x && cy == start.y)) {
        auto p = parent[cy][cx];
        cx = p.first; cy = p.second;
        if (cx == -1) break;
        path.push_back({cx, cy});
    }
    reverse(path.begin(), path.end());
    return path;
}

int main(int argc, char** argv) {
    // ===========================
    // CLI options (Monte Carlo)
    // ===========================
    // Usage:
    // auv_sim.exe [seed] [logfile]
    //
    // Example:
    // auv_sim.exe 123 logs/auv_log_123.csv
    int seed = 1337;
    string log_file = "logs/auv_log.csv";

    if (argc >= 2) {
        try { seed = stoi(argv[1]); } catch (...) {}
    }
    if (argc >= 3) {
        log_file = argv[2];
    }

    // --------------- Settings ---------------
    const double dt = 0.02;
    const double max_time = 120.0;

    const double target_depth = 5.0;
    const double safe_depth_limit = 8.0;
    const double min_battery_percent = 15.0;

    // --------------- Grid Map ---------------
    const int W = 30;
    const int H = 20;

    vector<vector<int>> occ(H, vector<int>(W, 0));

    // Static obstacles (same as before)
    for (int y = 6; y <= 14; y++) occ[y][12] = 1;
    for (int y = 2; y <= 8;  y++) occ[y][20] = 1;
    for (int x = 6; x <= 10; x++) occ[10][x] = 1;

    Cell start_cell{2, 2};
    Cell goal_cell{26, 16};

    auto cell_center = [](Cell c) {
        return pair<double,double>(c.x + 0.5, c.y + 0.5);
    };

    // --------------- Init AUV ---------------
    AUVState s;
    auto [sx, sy] = cell_center(start_cell);
    s.x = sx;
    s.y = sy;

    // --------------- Sensors ---------------
    mt19937 rng(seed);
    normal_distribution<double> depthNoise(0.0, 0.45);

    KalmanDepth kf;
    kf.depth = s.depth_true;
    kf.vel   = s.depth_vel;

    // --------------- Control ---------------
    PID pid{0.9, 0.10, 0.50};

    // --------------- Mission ---------------
    MissionState state = MissionState::ARM;
    double state_start_time = 0.0;

    int replan_count = 0;
    bool failsafe = false;
    string event = "none";

    auto transition_to = [&](MissionState next, const string& why="none") {
        state = next;
        state_start_time = s.t;
        pid.reset();
        event = why;
        cout << "[STATE] -> " << state_to_string(state) << " @ t=" << s.t << "s";
        if (why != "none") cout << " (" << why << ")";
        cout << "\n";
    };

    // --------------- Initial A* path ---------------
    vector<Cell> path = astar_path(occ, start_cell, goal_cell);
    int wp_idx = 0;

    if (path.empty()) {
        cout << "[A*] No path found from start to goal.\n";
        return 1;
    } else {
        cout << "[A*] Initial path length: " << path.size() << " waypoints\n";
    }

    // --------------- Logging ---------------
    filesystem::create_directories("logs");
    ofstream out(log_file);
    if (!out.is_open()) {
        cerr << "Failed to write " << log_file << "\n";
        return 1;
    }

    out << "t,mission_state,"
        << "depth_true,depth_meas,depth_est,target_depth,depth_error,u,"
        << "x,y,vx,vy,battery,"
        << "wp_idx,replan_count,event,seed\n";

    // Smoothing for planar velocity (low-pass)
    double vx_cmd = 0.0, vy_cmd = 0.0;

    // --------------- Main Loop ---------------
    while (s.t <= max_time && state != MissionState::DONE) {
        event = "none";

        // ---- measurement + kalman ----
        double depth_meas = s.depth_true + depthNoise(rng);
        kf.predict(dt);
        kf.update(depth_meas);
        double depth_est = kf.depth;

        // ---- failsafe ----
        if (!failsafe) {
            if (s.depth_true > safe_depth_limit) {
                failsafe = true;
                transition_to(MissionState::EMERGENCY_SURFACE, "FAILSAFE_DEPTH");
            }
            if (s.battery <= min_battery_percent) {
                failsafe = true;
                transition_to(MissionState::EMERGENCY_SURFACE, "FAILSAFE_BATTERY");
            }
        }

        // ---- mission ----
        if (state == MissionState::ARM) {
            vx_cmd = 0.0; vy_cmd = 0.0;
            if (s.t - state_start_time >= 1.0) {
                transition_to(MissionState::DIVE);
            }
        }
        else if (state == MissionState::DIVE) {
            vx_cmd = 0.0; vy_cmd = 0.0;
            if (abs(depth_est - target_depth) < 0.35) {
                transition_to(MissionState::NAVIGATE);
            }
        }
        else if (state == MissionState::NAVIGATE) {
            if (wp_idx >= (int)path.size()) {
                transition_to(MissionState::SURFACE, "GOAL_REACHED");
            } else {
                // ✅ LOOKAHEAD smoothing (pure pursuit)
                // Aim a few waypoints ahead to reduce stair-step behavior.
                int lookahead = 4;
                int idx = min(wp_idx + lookahead, (int)path.size() - 1);

                auto [wx, wy] = cell_center(path[idx]);
                double dx = wx - s.x;
                double dy = wy - s.y;
                double d = sqrt(dx*dx + dy*dy);

                // advance waypoint when close to current wp
                auto [cwx, cwy] = cell_center(path[wp_idx]);
                double dc = sqrt((cwx-s.x)*(cwx-s.x) + (cwy-s.y)*(cwy-s.y));
                if (dc < 0.35) wp_idx++;

                // speed command
                double speed = 1.25;
                if (d > 1e-6) {
                    vx_cmd = speed * (dx / d);
                    vy_cmd = speed * (dy / d);
                } else {
                    vx_cmd = 0.0; vy_cmd = 0.0;
                }

                // ✅ GUARANTEED replanning trigger at t > 15s
                // Inject a dynamic obstacle one time
                if (s.t > 15.0 && occ[11][17] == 0) {
                    occ[11][17] = 1;
                    transition_to(MissionState::REPLAN, "DYNAMIC_OBSTACLE_DISCOVERED");
                }
            }
        }
        else if (state == MissionState::REPLAN) {
            replan_count++;

            Cell cur{(int)floor(s.x), (int)floor(s.y)};
            cur.x = (int)clamp(cur.x, 0, W-1);
            cur.y = (int)clamp(cur.y, 0, H-1);

            vector<Cell> new_path = astar_path(occ, cur, goal_cell);
            if (new_path.empty()) {
                transition_to(MissionState::EMERGENCY_SURFACE, "REPLAN_FAILED");
            } else {
                path = new_path;
                wp_idx = 0;
                transition_to(MissionState::NAVIGATE, "REPLAN_OK");
            }
        }
        else if (state == MissionState::SURFACE) {
            vx_cmd = 0.0; vy_cmd = 0.0;
            if (depth_est < 0.15 && abs(s.depth_vel) < 0.05) {
                transition_to(MissionState::DONE);
            }
        }
        else if (state == MissionState::EMERGENCY_SURFACE) {
            vx_cmd = 0.0; vy_cmd = 0.0;
            if (depth_est < 0.15 && abs(s.depth_vel) < 0.05) {
                transition_to(MissionState::DONE);
            }
        }

        // ---- planar velocity smoothing ----
        double alpha = 0.12; // low pass filter factor
        s.vx = (1 - alpha) * s.vx + alpha * vx_cmd;
        s.vy = (1 - alpha) * s.vy + alpha * vy_cmd;

        // ---- depth control (using kalman estimate) ----
        double desired_depth = 0.0;
        if (state == MissionState::DIVE || state == MissionState::NAVIGATE || state == MissionState::REPLAN) {
            desired_depth = target_depth;
        }

        double depth_error = desired_depth - depth_est;
        double u = pid.step(depth_error, dt);
        u = clamp(u, -1.0, 1.0);

        if (state == MissionState::SURFACE || state == MissionState::EMERGENCY_SURFACE) {
            u = -1.0;
        }

        // ---- physics ----
        physics_step(s, u, dt);

        // ---- log ----
        out << s.t << ","
            << state_to_string(state) << ","
            << s.depth_true << ","
            << depth_meas << ","
            << depth_est << ","
            << desired_depth << ","
            << (desired_depth - depth_est) << ","
            << u << ","
            << s.x << ","
            << s.y << ","
            << s.vx << ","
            << s.vy << ","
            << s.battery << ","
            << wp_idx << ","
            << replan_count << ","
            << event << ","
            << seed
            << "\n";
    }

    out.close();
    cout << "\n✅ Done. Log saved to " << log_file << "\n";
    cout << "Replans triggered: " << replan_count << "\n";
    return 0;
}
