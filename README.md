# 🏎️ F1TENTH Autonomous Racing: Follow-the-Gap Reactive Controller

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy%20%7C%20Humble-3498DB?style=for-the-badge&logo=ros)
![Language](https://img.shields.io/badge/Language-Python%203-F39C12?style=for-the-badge&logo=python)
![Track](https://img.shields.io/badge/Track-Sepang%20International%20Circuit-E74C3C?style=for-the-badge)
![Group](https://img.shields.io/badge/Group-3-2ECC71?style=for-the-badge)
![Status](https://img.shields.io/badge/Challenge%201%20%26%202-Completed%20(100%25)-green?style=for-the-badge)

---

## 📋 Student & Course Metadata
- **Student Name:** Nevir Nafiz Molla
- **Course:** Autonomous Vehicles
- **Group:** Group 3
- **Assigned Track:** Sepang International Circuit
- **Primary Challenges:**
  1. 🌀 **Challenge 1:** 10 Consecutive Laps Without Collisions
  2. ⏱️ **Challenge 2:** Fastest Lap Time (without collisions)

---

## 🏆 Performance & Competition Results

The controller was benchmarked over 10 full consecutive laps on the official **Sepang International Circuit** map. The controller achieved **zero collisions** across all 10 laps with an exceptional fastest lap time.

| Lap Number | Lap Time (s) | Average Speed (m/s) | Track Distance (m) | Status |
|:---:|:---:|:---:|:---:|:---:|
| **Lap 1** | **79.78 s** | **6.04 m/s** | 481.6 m | ⭐ **FASTEST LAP** |
| **Lap 2** | 83.19 s | 5.80 m/s | 482.2 m | Clean (0 Collisions) |
| **Lap 3** | 81.52 s | 5.90 m/s | 481.3 m | Clean (0 Collisions) |
| **Lap 4** | 80.96 s | 5.95 m/s | 481.5 m | Clean (0 Collisions) |
| **Lap 5** | 82.99 s | 5.81 m/s | 482.5 m | Clean (0 Collisions) |
| **Lap 6** | 82.42 s | 5.85 m/s | 482.0 m | Clean (0 Collisions) |
| **Lap 7** | 82.88 s | 5.83 m/s | 482.8 m | Clean (0 Collisions) |
| **Lap 8** | 80.59 s | 5.98 m/s | 482.1 m | Clean (0 Collisions) |
| **Lap 9** | 82.25 s | 5.86 m/s | 482.3 m | Clean (0 Collisions) |
| **Lap 10** | 83.43 s | 5.79 m/s | 482.7 m | Clean (0 Collisions) |

- **Total Completed Laps:** 10 / 10 Consecutive Laps
- **Wall Collisions:** 0 (100% Collision-Free Navigation)
- **⚡ Fastest Lap Time:** **79.78 seconds**
- **⏱️ Average Lap Time:** **82.00 seconds**

---

## 🧠 Algorithmic Approach: High-Performance Follow-the-Gap

The vehicle navigates reactively using an advanced, multi-stage **Follow-the-Gap (FTG)** architecture with disparity extension, obstacle inflation, and curvature-adaptive speed profiling.

```
       [Raw 2D LiDAR Scan (/scan)]
                   │
                   ▼
     [1. Preprocessing & Moving Avg]
                   │
                   ▼
    [2. Disparity Extender Algorithm]
                   │
                   ▼
   [3. Dynamic Safety Bubble Inflation]
                   │
                   ▼
     [4. Widest Free-Space Gap Search]
                   │
                   ▼
     [5. Hybrid Target Point Selection]
     (68% Deepest Range + 32% Center)
                   │
                   ▼
    [6. Curvature-Adaptive Speed Profiler]
     (Steering & Slew-Rate Limiting)
                   │
                   ▼
    [AckermannDriveStamped (/drive)]
```

### 1. LiDAR Preprocessing & Noise Filtering
Raw 2D LiDAR ranges ($1080$ beams over a $270^\circ$ FOV) are processed:
- Invalid measurements (`NaN`, `inf`, or negative readings) are replaced with maximum perception distance ($8.6\text{ m}$).
- A $5$-point moving average smoothing filter removes high-frequency beam jitter without losing edge fidelity:
  $$r_{smoothed}[i] = \frac{1}{5}\sum_{k=-2}^{2} r[i+k]$$
- The field of view is focused onto the forward driving sector ($\pm 71^\circ$), ignoring irrelevant rear/blind spots.

### 2. Disparity Extender Algorithm
Standard FTG suffers from "corner clipping" when entering sharp turns, because raw laser beams see deep space behind a wall corner. The disparity extender detects large depth steps between adjacent beams:
$$|\Delta r| = |r[i+1] - r[i]| > \Delta r_{threshold} \quad (\text{with } \Delta r_{threshold} = 0.28\text{ m})$$
When a disparity is found at distance $r_{close}$, an angular buffer is calculated using the vehicle's safe half-width $W_{margin} = 0.42\text{ m}$:
$$\theta_{ext} = \arctan\left(\frac{W_{margin}}{r_{close}}\right)$$
All beams within $\theta_{ext}$ on the further side of the edge are clamped to $r_{close}$, preventing the vehicle from turning into walls.

### 3. Dynamic Safety Bubble Inflation
The closest physical obstacle within the forward arc is identified at distance $d_{min}$. A safety bubble of radius $R_{bubble} = 0.46\text{ m}$ is projected around it:
$$\theta_{bubble} = \arctan\left(\frac{R_{bubble}}{d_{min}}\right)$$
All laser beams within $[i_{closest} - \frac{\theta_{bubble}}{\Delta\theta}, i_{closest} + \frac{\theta_{bubble}}{\Delta\theta}]$ are set to $0.0\text{ m}$, preventing any trajectory from entering the vehicle's collision envelope.

### 4. Widest Gap Identification
Contiguous sequences of non-zero beams exceeding the minimum navigable gap distance ($r > 1.10\text{ m}$) are identified. The widest contiguous gap $[i_{start}, i_{end}]$ is selected.

### 5. Hybrid Goal Point Selection
Instead of naively selecting the center of the gap (which drives conservatively in the middle) or only the deepest point (which can oscillate near corners), a hybrid target point is computed:
$$i_{target} = \text{round}\left(0.68 \cdot i_{deepest} + 0.32 \cdot i_{center}\right)$$
This enables the vehicle to take an optimal racing line, clip turn apexes cleanly, and maintain higher cornering speeds.

### 6. Curvature-Adaptive Speed Profiler
Speed is dynamically regulated based on absolute steering angle $|\delta|$ and forward line-of-sight distance $d_{front}$:
- **Straightaways** ($|\delta| < 2.9^\circ, d_{front} > 5.5\text{ m}$): **$8.2\text{ m/s}$**
- **Gentle Sweepers** ($|\delta| < 5.7^\circ, d_{front} > 4.0\text{ m}$): **$6.0\text{ m/s}$**
- **Medium Turns** ($|\delta| < 9.7^\circ$): **$4.4\text{ m/s}$**
- **Tight Chicanes** ($|\delta| < 14.3^\circ$): **$3.1\text{ m/s}$**
- **Hairpins** ($|\delta| \ge 14.3^\circ$): **$2.4\text{ m/s}$**

Longitudinal acceleration is governed by a slew-rate limiter ($a_{max} = 7.5\text{ m/s}^2, d_{max} = 8.8\text{ m/s}^2$) to preserve tire traction and prevent spin-outs.

---

## 🔄 Lap Counter & ⏱️ Lap Timer Implementation

The controller includes an automated, high-precision lap counting and timing subsystem subscribing to `/ego_racecar/odom`.

### Start/Finish Line Detection on Sepang Circuit
- **Line Origin:** $(x_0, y_0) = (0.0, 0.0)$
- **Track Heading at Start Line:** $\theta_0 = -3.05692\text{ rad}$ ($-175.1^\circ$)
- **Forward Vector:** $\vec{u}_{fwd} = (\cos\theta_0, \sin\theta_0)$
- **Lateral Vector:** $\vec{u}_{lat} = (-\sin\theta_0, \cos\theta_0)$

The car's position $(x, y)$ is projected onto the track's longitudinal axis:
$$s = x \cos\theta_0 + y \sin\theta_0$$
$$d_{lat} = |-x \sin\theta_0 + y \cos\theta_0|$$

A lap completion is triggered when:
1. $s_{prev} < 0.0$ and $s_{curr} \ge 0.0$ (sign transition across the start line plane)
2. $d_{lat} < 3.0\text{ m}$ (within the track corridor)
3. Cumulative lap distance $D_{lap} > 350.0\text{ m}$ (hysteresis cooldown preventing false triggers at startup)

### Live Visual Feedback
1. **Real-Time RViz 3D Text HUD (`/lap_marker`):**
   A bright gold 3D text marker floats $1.8\text{ m}$ above the racecar in RViz:
   ```
   LAP: 3/10 | TIME: 42.6s | BEST: 79.78s
   ```
2. **Terminal Telemetry (1 Hz):**
   Live status updates stream to the terminal while driving:
   ```
   ⏱️  [LAP  1/10] Current Time:  14.2 s | Speed:  7.5 m/s | Best: N/A
   ```
3. **Lap Completion Banner:**
   When crossing the line, a formatted banner outputs lap time, average speed, distance, and flags new fastest laps.

---

## 📁 Repository Structure

```
d:\nebirAssaignment\
├── benchmark_lap.py               # Fast headless 10-lap benchmark script
├── runsim                         # Linux/WSL one-command launcher
├── runsim.bat                     # Windows CMD/PowerShell one-command launcher
├── runsim.ps1                     # PowerShell executable script
├── README.md                      # Complete project documentation
├── .gitignore                     # Build & cache ignore rules
├── Laboratories/                  # Course laboratory tutorials (Labs 1-7)
├── img/                           # Architectural & topic diagrams
├── videos/                        # Reference video guides
└── src/
    ├── controllers/               # Autonomous navigation controllers package
    │   ├── controllers/
    │   │   ├── gap_node.py        # Follow-the-Gap reactive controller with Lap Counter/Timer
    │   │   ├── mpc_node.py        # Model Predictive Controller template
    │   │   ├── pid_node.py        # Wall Follower PID template
    │   │   ├── purepursuit_node.py# Pure Pursuit path tracker template
    │   │   ├── rrt_node.py        # RRT local path planner template
    │   │   └── ttc_node.py        # Automatic Emergency Braking (AEB) node
    │   ├── package.xml
    │   └── setup.py
    ├── f1tenth_gym_ros/           # ROS 2 simulation bridge package
    │   ├── config/
    │   │   └── sim.yaml           # Sepang circuit parameters & start pose
    │   ├── f1tenth_gym_ros/
    │   │   └── gym_bridge.py      # Simulator bridge node
    │   ├── launch/
    │   │   ├── gym_bridge_launch.py
    │   │   └── gym_bridge.rviz    # Pre-configured RViz with Lap HUD & Sepang camera
    │   └── maps/
    │       ├── Sepang_map.png     # Sepang high-resolution circuit map
    │       └── Sepang_map.yaml    # Sepang map metadata and resolution
    ├── path_planning/             # Global path planning utilities
    └── waypoints/
        ├── Sepang_centerline.csv  # Sepang track centerline coordinates
        └── Sepang_raceline.csv    # Sepang optimal racing line
```

---

## 🚀 Execution Instructions

### Quick Start: One-Command Simulation (`runsim`)

From the project root directory (`d:\nebirAssaignment`), simply run:

#### In Windows PowerShell or Command Prompt:
```powershell
runsim
```
*(or `.\runsim.bat` or `.\runsim.ps1`)*

#### In WSL Ubuntu Terminal:
```bash
./runsim
```

**What this shortcut does:**
1. Automatically sources ROS 2 and local packages.
2. Compiles the workspace using `colcon` if not already built.
3. Launches the F1TENTH simulator and opens RViz2 with the top-down view centered on the Sepang circuit.
4. Starts the `gap_node` controller with the live Lap Counter, Lap Timer, and 3D floating HUD.
5. Streams all real-time telemetry and lap records directly to your terminal.

---

### Running the Fast Headless Benchmark

To verify 10 consecutive laps and test lap times without opening the RViz GUI:

```bash
# In WSL or Linux terminal:
cd /mnt/d/nebirAssaignment
python3 benchmark_lap.py
```

---

### Manual Step-by-Step Launch (Alternative)

If you prefer launching each component in separate terminals:

**Terminal 1 — Simulator & RViz2:**
```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/d/nebirAssaignment
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 — Follow-the-Gap Controller:**
```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/d/nebirAssaignment
source install/setup.bash
ros2 run controllers gap_node
```

---

## 🎥 Video Evidence Submission Checklist

To achieve the full **2 mandatory points** for video evidence:
- [x] **Vehicle Driving on Sepang:** The vehicle navigates the entire track smoothly without hitting walls.
- [x] **Lap Counter Clearly Visible:** Displayed in both the terminal output (`[LAP X/10]`) and floating above the car in RViz (`LAP: X/10`).
- [x] **Lap Timer Displayed:** Real-time elapsed time shown live, with per-lap completion records logged on the terminal.

---

## 🗣️ Oral Project Defense Preparation

Be prepared to answer these core questions during the individual oral defense:

1. **Why is the Disparity Extender necessary?**
   - *Answer:* In raw LiDAR data, a corner creates a depth jump where rays pass into empty space behind the corner. Naive Follow-the-Gap sees this as an open gap and steers directly into the inner apex wall. The disparity extender detects this jump and expands the nearer obstacle by the car's half-width ($W_{margin} = 0.42\text{ m}$), eliminating corner clipping.

2. **How does the Safety Bubble work?**
   - *Answer:* The node locates the closest LiDAR return within the forward driving arc. It computes the angular span subtended by a safety radius ($R_{bubble} = 0.46\text{ m}$) at that distance ($\theta = \arctan(R/d)$) and sets all rays within that cone to zero. This ensures the widest-gap search ignores any space that would bring the vehicle dangerously close to walls.

3. **How is the target point selected within the gap?**
   - *Answer:* We use a weighted combination: $68\%$ of the deepest LiDAR point within the gap (which pulls the car forward through open straights and apexes) and $32\%$ of the geometric center (which guarantees clearance from flanking obstacles).

4. **How are laps detected reliably without false positives?**
   - *Answer:* Using vehicle odometry, coordinates $(x, y)$ are projected onto the track's forward vector at the start line ($s = x\cos\theta + y\sin\theta$). Crossing occurs when $s$ transitions from negative to positive within lateral corridor $|d_{lat}| < 3.0\text{ m}$, guarded by a $350\text{ m}$ distance hysteresis cooldown to prevent multiple triggers.
