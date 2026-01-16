use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};

fn mean(vals: &[f64]) -> f64 {
    if vals.is_empty() {
        return 0.0;
    }
    vals.iter().sum::<f64>() / vals.len() as f64
}

fn rmse(vals: &[f64]) -> f64 {
    if vals.is_empty() {
        return 0.0;
    }
    let mse = vals.iter().map(|x| x * x).sum::<f64>() / vals.len() as f64;
    mse.sqrt()
}

fn main() {
    let path = "../logs/auv_log.csv";
    let file = File::open(path).expect("Could not open log file. Run C++ sim first.");
    let reader = BufReader::new(file);

    // NEW CSV format:
    // t,mission_state,
    // depth_true,depth_meas,depth_est,target_depth,depth_error,u,
    // x,y,vx,vy,battery,
    // wp_idx,replan_count,event

    let mut max_depth_true: f64 = 0.0;
    let mut max_depth_est: f64 = 0.0;

    let mut min_battery: f64 = 100.0;
    let mut start_battery: Option<f64> = None;
    let mut end_battery: Option<f64> = None;

    let mut final_time: f64 = 0.0;

    let mut last_state: Option<String> = None;
    let mut last_t: Option<f64> = None;

    // time per state
    let mut state_time: HashMap<String, f64> = HashMap::new();

    // metrics during NAVIGATE
    let mut nav_abs_errors: Vec<f64> = vec![];
    let mut nav_sq_errors_true: Vec<f64> = vec![];

    // total replans
    let mut max_replans: i32 = 0;

    // count how many times we entered REPLAN
    let mut replan_entries: i32 = 0;

    for (i, line) in reader.lines().enumerate() {
        let line = line.unwrap();
        if i == 0 {
            continue; // header
        }

        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < 16 {
            continue;
        }

        let t: f64 = parts[0].parse().unwrap_or(0.0);
        let state = parts[1].to_string();

        let depth_true: f64 = parts[2].parse().unwrap_or(0.0);
        let depth_est: f64 = parts[4].parse().unwrap_or(0.0);
        let target_depth: f64 = parts[5].parse().unwrap_or(0.0);
        let depth_error: f64 = parts[6].parse().unwrap_or(0.0);

        let battery: f64 = parts[12].parse().unwrap_or(0.0);
        let replan_count: i32 = parts[14].parse().unwrap_or(0);

        // battery start/end
        if start_battery.is_none() {
            start_battery = Some(battery);
        }
        end_battery = Some(battery);

        if battery < min_battery {
            min_battery = battery;
        }

        // max depths
        if depth_true > max_depth_true {
            max_depth_true = depth_true;
        }
        if depth_est > max_depth_est {
            max_depth_est = depth_est;
        }

        // replans
        if replan_count > max_replans {
            max_replans = replan_count;
        }

        // count REPLAN entries
        if last_state.as_deref() != Some(&state) && state == "REPLAN" {
            replan_entries += 1;
        }

        // state timing accumulation
        if let (Some(prev_state), Some(prev_t)) = (&last_state, last_t) {
            let dt = t - prev_t;
            if dt > 0.0 && dt < 0.5 {
                *state_time.entry(prev_state.clone()).or_insert(0.0) += dt;
            }
        }

        // nav metrics
        if state == "NAVIGATE" {
            nav_abs_errors.push(depth_error.abs());

            // RMSE using TRUE depth vs target depth
            let e_true = target_depth - depth_true;
            nav_sq_errors_true.push(e_true);
        }

        // update trackers
        last_state = Some(state);
        last_t = Some(t);
        final_time = t;
    }

    let start_b = start_battery.unwrap_or(100.0);
    let end_b = end_battery.unwrap_or(min_battery);
    let battery_used = (start_b - end_b).max(0.0);

    let nav_mae = mean(&nav_abs_errors);
    let nav_rmse_true = rmse(&nav_sq_errors_true);

    println!("\n===== AUV Autonomy Report (Rust) =====");
    println!("Mission end time: {:.2} s", final_time);

    println!("\n--- Depth Safety ---");
    println!("Max depth (true): {:.3} m", max_depth_true);
    println!("Max depth (Kalman est): {:.3} m", max_depth_est);

    println!("\n--- Battery ---");
    println!("Min battery observed: {:.2} %", min_battery);
    println!(
        "Battery used: {:.2} % (start {:.2} -> end {:.2})",
        battery_used, start_b, end_b
    );

    println!("\n--- Navigation Control (NAVIGATE state) ---");
    println!("Depth MAE (using Kalman error): {:.3} m", nav_mae);
    println!("Depth RMSE (true vs target): {:.3} m", nav_rmse_true);

    println!("\n--- Replanning ---");
    println!("Replan count (max seen in log): {}", max_replans);
    println!("REPLAN state entries: {}", replan_entries);

    println!("\n--- Time Spent Per State ---");
    // print in a nice order
    let order = vec![
        "ARM", "DIVE", "NAVIGATE", "REPLAN", "SURFACE", "EMERGENCY_SURFACE", "DONE",
    ];
    for st in order {
        let v = *state_time.get(st).unwrap_or(&0.0);
        if v > 0.0 {
            println!("{:<18} {:.2} s", st, v);
        }
    }

    println!("\n✅ Report complete.\n");
}
