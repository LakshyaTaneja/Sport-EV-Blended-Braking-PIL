# Sport-EV-Blended-Braking-PIL

### Simulation-Based Design and Processor-in-the-Loop (PIL) Validation of a Blended Braking System for a Sport Electric Vehicle (EV) using an ARM Cortex-M4 Processor.

**Author:** Lakshay Taneja
**Hardware:** STM32 Nucleo-F446RE | **Software:** MATLAB/Simulink 2025b

<p align="center">
  <img src="https://github.com/user-attachments/assets/140e4cd1-3c1a-4703-a4e2-17e149621bbd" width="45%" alt="Hardware Setup" />
  <img src="https://github.com/user-attachments/assets/7302784c-1c07-43e4-bb2a-ced97ed999ea" width="45%" alt="Simulink Model" />
</p>

## Project Overview
This project designs and validates a safety-critical **Blended Braking Controller** for a Sport Electric Vehicle (EV). The system dynamically splits torque demand between the **Electric Motor (Regenerative Braking)** and an **Electro-Mechanical Brake-by-Wire (BBW) Actuator** to maximize energy recovery while ensuring vehicle stability.

The control logic was validated using **Processor-in-the-Loop (PIL)** methodology, running the compiled C-code on an **ARM Cortex-M4 (STM32F446RE)** while interacting with a high-fidelity vehicle physics model running in Simulink.

## Key Engineering Features
* **Sport EV Tuning:** Calibrated for a 1600kg vehicle with low drag ($C_d=0.22$) and high-performance braking ($1.0g$).
* **One-Pedal Driving:** Implements a calibrated **-300 Nm** regenerative drag upon throttle lift-off.
* **Hardware-in-the-Loop (HIL) Inputs:** Integrated physical potentiometers (Accelerator/Brake) via ADC to drive the simulation in real-time.
* **Embedded Optimization:** Used **Low-Layer (LL)** drivers and **Single-Precision (FPU)** arithmetic to achieve a deterministic **100Hz** control loop.

## System Architecture
### 1. The Plant (Vehicle Physics)
* **Dynamics:** Forward-facing $F=ma$ model including aerodynamic drag and rolling resistance.
* **Battery:** 50Ah High-Voltage Buffer Pack modeled using Thevenin Equivalent Circuit (Dynamic $V_{oc}$ and $R_{int}$).
* **Actuators:** Modeled delays for Brake-by-Wire (10ms) and Motor Inverter.

### 2. The Controller (STM32F446RE)
* **State Machine:** Arbitrates between propulsion, coasting, and braking states.
* **Safety Logic:**
    * **Speed Governor:** Hard cap at 45 m/s (162 km/h).
    * **Regen Constraints:** Derates torque at high RPM (Field Weakening) and high SoC (>95%).
    * **Signal Conditioning:** 5% Deadband to filter ADC noise.

## Validation Results

### Scenario A: High Speed Coasting
* **Action:** Accelerate to 40 m/s, then release throttle.
* **Result:** Smooth deceleration; Battery SoC **RISES** (Kinetic $\to$ Chemical Energy).

<p align="center">
  <img src="https://github.com/user-attachments/assets/793e0a98-b76b-48c5-bbc9-30ac14e6426a" width="80%" alt="Vehicle Speed Profile" />
</p>

### Scenario B: Panic Stop
* **Action:** Apply 100% Brake at 155 km/h.
* **Result:** Regen saturates at -2500 Nm; Friction spikes to **-4000 Nm**; Vehicle stops in < 4.5s ($1.0g$).

<p align="center">
  <img src="https://github.com/user-attachments/assets/bd4e6340-0bba-4471-964f-b823c64041c1" width="90%" alt="Panic Stop Graphs" />
</p>

## How to Run This Project

### Prerequisites
* MATLAB/Simulink with Embedded Coder & STM32 Support Package.
* STM32CubeMX (for hardware configuration).
* STM32 Nucleo-F446RE Board.

### Setup Steps
1.  **Hardware:** Connect the Nucleo board via USB. Connect 10kΩ potentiometers to **PA0** (Accel) and **PA1** (Brake).
2.  **Build:** Open `01_PIL_Build_Environment/PIL_System_POTS.slx`. Build the subsystem to generate the PIL block.
3.  **Run:** Open `02_Vehicle_Simulation/PRO_POTS.slx`. Ensure the PIL block is linked. Click **Run**.

---
*Project developed for the Master of Technology program at the Indian Institute of Science (IISc).*
