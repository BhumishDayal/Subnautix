# AUV Mission Simulator (C++ + Rust + Python)

A lightweight Autonomous Underwater Vehicle (AUV) mission simulator demonstrating:
- A **finite-state mission planner**: **DIVE → HOLD → SURFACE**
- A **PID depth-hold controller** running at **50Hz**
- **Failsafe emergency surfacing** on unsafe depth or low battery
- Telemetry logging → offline analysis + visualization

This project is designed to be **junior-friendly** but still **engineering-grade** for robotics/autonomy internships.

---

## Project Architecture

**C++ (Simulation + Controller)**  
✅ Runs the mission and produces a CSV telemetry log:
- depth, target depth, error
- thrust command `u`
- battery estimate
- mission state + failsafe reason

**Rust (Telemetry Metrics / Evaluation)**  
✅ Reads CSV logs and prints mission KPIs:
- overshoot (%)
- mean absolute error (HOLD phase)
- steady-state error
- settling time
- battery usage
- time-to-surface

**Python (Visualization)**  
✅ Plots mission behavior in one window:
- Depth vs target depth
- Depth error
- Control thrust + battery
- Mission transition markers

---

## Folder Structure
AUV/
cpp_sim/
main.cpp
CMakeLists.txt
rust_analyzer/
src/main.rs
Cargo.toml
python_plot/
plot.py
logs/
auv_log.csv (generated)
.vscode/
tasks.json

---

## Autonomy Report Output

This report is auto-generated after each run:

![AUV Report](plots/report.png)

## How To Run (One Command)

Run this from the **AUV/** folder in PowerShell:

```powershell
cmake -S cpp_sim -B cpp_sim/build; cmake --build cpp_sim/build; & .\cpp_sim\build\Debug\auv_sim.exe; Push-Location rust_analyzer; cargo run --release; Pop-Location; Push-Location python_plot; python plot.py; Pop-Location
