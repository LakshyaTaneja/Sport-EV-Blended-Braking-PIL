# Blended Braking Controller for Sport EV (STM32 PIL)

**Institute:** Indian Institute of Science (IISc), Bangalore  
**Status:** Completed (October 2025)

---

## Project Overview

This project implements a safety-critical **Blended Braking Controller** for a high-performance Sport Electric Vehicle (~1600 kg).
The controller maximizes **regenerative braking energy recovery** while ensuring **consistent vehicle deceleration** by dynamically blending regenerative and friction braking.

The control logic is designed in **Simulink**, auto-generated into **C code**, and validated on an **STM32F446RE** microcontroller using:

* **Processor-in-the-Loop (PIL)**
* **Hardware-in-the-Loop (HIL)**

---

## Key Features

### Dynamic Torque Arbitration

* Prioritizes motor torque for regenerative braking
* Seamlessly compensates with friction braking to meet driver deceleration demand

### Safety-Critical Logic

* **Zero-Speed Clamp:** Prevents reverse motion at standstill (|v| < 0.1 m/s)
* **Field Weakening Limit:** Torque interpolation limits at high vehicle speeds (~45 m/s) to protect the motor
* **State-of-Charge Protection:** Regenerative braking disabled when battery SoC exceeds 95%

### Vehicle & Battery Modeling

* **Sport EV Dynamics:** Validated 0–162 km/h acceleration in **9.0 seconds**
* **Battery Model:** Thevenin equivalent circuit with **Coulomb counting** for accurate SoC estimation

---

## Repository Structure

```text
├── Docs/               # Full project report and presentation slides
├── Simulink_Model/     # Vehicle plant model and ECU state machine
├── STM32_PIL/          # Embedded Coder generated files (ARM Cortex-M4)
└── Results/            # Validation plots (emergency braking, coasting, etc.)
```

---

## System Architecture

The system consists of:

* A forward-facing vehicle plant model
* A discrete-time embedded controller deployed on STM32

Detailed mathematical models and control logic are documented in:

```
Docs/Report.pdf
```

---

## Validation Results

### 1. Emergency Braking (Panic Stop)

**Scenario:** Driver applies 100% brake at 43 m/s (155 km/h)

* **Regenerative Torque:** Ramps linearly from −1200 Nm to −2500 Nm as speed drops
  (Field-Weakening-Aware Logic)
* **Friction Torque:** Inverse compensation maintains constant deceleration (~1.0 g)

---

### 2. One-Pedal Driving

**Scenario:** Driver releases throttle at 40 m/s

* Vehicle decelerates with approximately **−300 Nm** of regenerative drag
* **Battery SoC:** Increases, confirming kinetic-to-chemical energy recovery

---

## Tech Stack & Hardware

* **Simulation:** MATLAB R2024b, Simulink, Simscape
* **Code Generation:** Simulink Embedded Coder
* **Target Hardware:** STM32 Nucleo-F446RE
  (ARM Cortex-M4 @ 180 MHz)
* **HIL Interface:** 10 kΩ potentiometers for accelerator and brake inputs via ADC

---

## How to Run

### 1. Clone the Repository

```bash
git clone https://github.com/LakshyaTaneja/Sport-EV-Blended-Braking-PIL.git
```

### 2. Generate ECU_Controller PIL Block.

* Open `ECU_Controller.slx`
* Build it to generate "ECU_Controller" PIL Block.
* Copy the generated "ECU_Controller" PIL Block.


### 3. Processor-in-the-Loop (PIL) Validation

* Connect STM32 Nucleo-F446RE via USB
* Open `EV_Plant_Model.slx` and paste the generated "ECU_Controller" PIL Block.
* Run the simulation

---

## Author

**Lakshay Taneja**  
Department of Mechanical Engineering  
Indian Institute of Science (IISc), Bangalore

---