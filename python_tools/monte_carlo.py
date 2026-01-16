import os
import csv
import subprocess
import statistics

EXE_PATH = r"..\cpp_sim\build\Debug\auv_sim.exe"
LOG_DIR = r"..\logs"

N_RUNS = 30  # increase to 50/100 if you want
TARGET_DEPTH = 5.0

def parse_log(filepath):
    rows = []
    with open(filepath, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    if not rows:
        return None

    # success if last state == DONE
    last_state = rows[-1]["mission_state"]
    success = (last_state == "DONE")
    end_time = float(rows[-1]["t"])

    start_batt = float(rows[0]["battery"])
    end_batt = float(rows[-1]["battery"])
    battery_used = max(0.0, start_batt - end_batt)

    max_depth_true = max(float(r["depth_true"]) for r in rows)

    # RMSE during NAVIGATE (true depth vs target)
    nav_errs = []
    for r in rows:
        if r["mission_state"] == "NAVIGATE":
            e = TARGET_DEPTH - float(r["depth_true"])
            nav_errs.append(e)

    rmse = 0.0
    if nav_errs:
        rmse = (sum(e*e for e in nav_errs) / len(nav_errs)) ** 0.5

    replans = max(int(r["replan_count"]) for r in rows)

    return {
        "success": int(success),
        "end_time": end_time,
        "battery_used": battery_used,
        "max_depth_true": max_depth_true,
        "nav_rmse": rmse,
        "replans": replans,
    }

def main():
    os.makedirs(LOG_DIR, exist_ok=True)

    results = []
    success_count = 0

    for i in range(N_RUNS):
        seed = 1000 + i
        log_file = os.path.join(LOG_DIR, f"auv_log_{seed}.csv")

        # run sim
        cmd = [EXE_PATH, str(seed), log_file]
        subprocess.run(cmd, check=True)

        metrics = parse_log(log_file)
        if metrics is None:
            continue

        metrics["seed"] = seed
        results.append(metrics)
        success_count += metrics["success"]

    # summary
    if not results:
        print("No results found.")
        return

    success_rate = success_count / len(results) * 100.0
    print("\n===== Monte Carlo Summary =====")
    print(f"Runs: {len(results)}")
    print(f"Success rate: {success_rate:.2f}%")

    def avg(key):
        vals = [r[key] for r in results]
        return statistics.mean(vals)

    print(f"Avg end time: {avg('end_time'):.2f} s")
    print(f"Avg battery used: {avg('battery_used'):.2f} %")
    print(f"Avg NAV RMSE: {avg('nav_rmse'):.3f} m")
    print(f"Avg replans: {avg('replans'):.2f}")

    # save CSV
    out_csv = os.path.join(LOG_DIR, "monte_carlo_summary.csv")
    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["seed","success","end_time","battery_used","max_depth_true","nav_rmse","replans"])
        writer.writeheader()
        for r in results:
            writer.writerow(r)

    print(f"\n✅ Saved: {out_csv}")

if __name__ == "__main__":
    main()
