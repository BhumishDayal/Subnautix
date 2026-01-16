# Subnautix - AUV Mission Simulator (C++ + Rust + Python)

A lightweight Autonomous Underwater Vehicle (AUV) mission simulator demonstrating:
- A **finite-state mission planner**: **DIVE → HOLD → SURFACE**
- A **PID depth-hold controller** running at **50Hz**
- **Failsafe emergency surfacing** on unsafe depth or low battery
- Telemetry logging → offline analysis + visualization

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

## How To Run (One Command)

Run this from the **AUV/** folder in PowerShell:

```powershell
cmake -S cpp_sim -B cpp_sim/build; cmake --build cpp_sim/build; & .\cpp_sim\build\Debug\auv_sim.exe; Push-Location rust_analyzer; cargo run --release; Pop-Location; Push-Location python_plot; python plot.py; Pop-Location
