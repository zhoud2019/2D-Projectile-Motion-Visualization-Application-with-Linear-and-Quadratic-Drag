# 🎯 2D Projectile Motion Simulator

A physics-based desktop application built with Python and Tkinter that simulates and visualizes projectile motion under three different drag models in real time.

---

## 📸 Screenshots

**Input Panel**

![Input screen](https://github.com/user-attachments/assets/ab4e2050-c50a-493f-9443-712d180f7e5a)

**Simulation in Progress**

![Simulation mid-flight](https://github.com/user-attachments/assets/944e0bbe-613a-41c8-8ae5-7b530f7eef10)

![Simulation trails](https://github.com/user-attachments/assets/ce1e8129-714a-4c23-b7f7-c2b5fbf4b033)

**Results Screen**

![Results summary](https://github.com/user-attachments/assets/0eb0e6e3-a873-4416-a57b-df2fd75c3953)

---

## 📖 Overview

This simulator lets you launch a projectile with a chosen velocity, angle, and mass, then watch three trajectories animate simultaneously:

- 🔵 **Ideal** — No air resistance (analytical solution)
- 🔴 **Linear Drag** — Drag force proportional to velocity
- 🟢 **Quadratic Drag** — Drag force proportional to velocity² using the Linear Algebraic Trajectory (LAT) approximation

After the animation completes, you can view a full results breakdown comparing range, maximum height, and flight time across all three models.

---

## ✨ Features

- Real-time animated trajectories with color-coded trails
- Three physics models computed and displayed simultaneously
- Analytical and numerical methods combined for accuracy
- Clean results summary screen with a back button to re-run
- Scales automatically to any screen resolution

---

## 🧪 Physics Models

### Ideal (No Drag)

Uses the standard kinematic equations:

$$x(t) = v_0 \cos(\alpha)\, t$$
$$y(t) = v_0 \sin(\alpha)\, t - \frac{1}{2}g t^2$$

Range, max height, and flight time are computed analytically.

---

### Linear Drag

Drag force is modeled as $F_d = k v$, where $k$ is derived from the projectile diameter and mass:

$$k = C \cdot v_0, \quad C = \frac{0.22 \cdot D^2}{m}$$

- Ascent time is solved analytically
- Peak height is computed from the ascent equation
- Descent time is found iteratively using the **gamma function approximation**
- Horizontal range uses the exponential decay formula for $x(t)$

---

### Quadratic Drag (LAT Approximation)

Drag force is modeled as $F_d = C v^2$. The **Linear Algebraic Trajectory (LAT)** approximation replaces the full nonlinear ODE with a tractable closed-form solution:

$$x(t) = \frac{1}{C} \ln(C v_{x0}\, t + 1)$$

$$y(t) = \left(w_0 + \frac{g}{2a}\right)\frac{\ln(1 + at)}{a} - \frac{gt^2}{4} - \frac{gt}{2a}$$

where $a = C \cdot v_{x0}$.

- Ascent time is solved using the arctangent formula
- Landing time is found via **binary search** (bisection method)

---

## 🚀 Getting Started

### Prerequisites

- Python 3.7+
- Tkinter (included with most Python installations)

To verify Tkinter is available:

```bash
python -m tkinter
```

If a small window appears, you're good to go.

### Installation

```bash
git clone https://github.com/your-username/projectile-simulator.git
cd projectile-simulator
```

No additional packages are required — the project uses only the Python standard library.

### Running the App

```bash
python projectile_simulator.py
```

---

## 🖥️ How to Use

1. **Enter the initial velocity** (m/s)
2. **Enter the launch angle** (degrees from horizontal)
3. **Enter the mass** of the projectile (kg)
4. Click **"Click to enter values!"** to set up the simulation
5. Click **"Run the simulation!"** to launch the animation
6. Once all three balls land, click **"Show Results"** to view the comparison table
7. Click **"Back"** to return and run another simulation

---

## 📐 Parameters

| Parameter | Description | Units |
|---|---|---|
| Initial Velocity | Launch speed of the projectile | m/s |
| Launch Angle | Angle above the horizontal | degrees |
| Mass | Mass of the projectile | kg |
| Diameter (fixed) | Used to compute drag coefficient | 0.25 m |
| Gravity (fixed) | Standard gravitational acceleration | 9.81 m/s² |

> **Note:** The projectile diameter is currently fixed at 0.25 m internally. Future versions may expose this as a user input.

---

## 📊 Output Metrics

For each drag model, the simulator reports:

| Metric | Description |
|---|---|
| Range | Horizontal distance traveled (m) |
| Max Height | Peak altitude reached (m) |
| Flight Time | Total time in the air (s) |

---

## 🗂️ Project Structure

```
projectile-simulator/
│
├── projectile_simulator.py   # Main application file
└── README.md
```

---

## 🔧 Known Limitations

- The projectile diameter is hardcoded at 0.25 m; changing mass alone affects the drag coefficient but not the geometry
- The LAT approximation for quadratic drag is an approximation — results diverge from numerical integration at very high drag or shallow angles
- The app currently supports launch from ground level only (y₀ = 0)

---

## 🛣️ Potential Improvements

- [ ] Add user input for projectile diameter
- [ ] Add numerical ODE integration (e.g., RK4) as a fourth comparison model
- [ ] Allow launch from an elevated height
- [ ] Export results to CSV
- [ ] Add wind effects

---

## 📚 References

- Borghi, R. (2013). *On the Linear Algebraic Trajectory Approximation for Projectile Motion.* European Journal of Physics.
- Taylor, J. R. *Classical Mechanics.* University Science Books.
- Standard drag model references for linear and quadratic regimes.

---

## 📄 License

This project is open source. Feel free to use, modify, and share it.
