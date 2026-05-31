# Autonomous Cyber-Physical Thermal Optimizer (ACPTO)

The Autonomous Cyber-Physical Thermal Optimizer (ACPTO) is an embedded, edge-computing research platform designed to explore the intersection of machine learning, low-level computer architecture, and classical control theory.

The goal of this project is to build a hardware-in-the-loop testbed that can autonomously map out and tune its own physical thermodynamic control system in real time without human intervention.

## Why I am Building This

Deep-space exploration payloads and advanced laboratory instruments face volatile environments where thermal drift can quickly compromise sensitive sensors. Traditional architectures rely on static control parameters that cannot adapt to hardware degradation or unmodeled atmospheric shocks. Furthermore, running heavy statistical optimization models typically requires high-power desktop infrastructure.

This project serves as a proof-of-concept for moving complex statistical inference directly onto low-resource edge hardware. By bypassing some libraries and optimizing the core mathematical bottlenecks at the register level, I aim to build a system with self-healing autonomy that runs entirely on localized silicon.

## System Architecture & Theory

The project runs on a Teensy 4.0 (NXP iMXRT1062, ARM Cortex-M7 at 600 MHz) with a strict separation of concerns between two concurrent execution contexts:

* **The Deterministic Control Plant:** A high-frequency, time-critical PID loop executes via a hardware-timer interrupt (`IntervalTimer`) to actively manage the physical temperature of an aluminum mass, guaranteeing deterministic sampling intervals independent of the optimizer's workload.
* **The Mathematical Coprocessor:** The main execution loop runs a localized 2D Gaussian Process (GP) regression with a Radial Basis Function (RBF) kernel to model the performance landscape across proportional ($K_p$) and derivative ($K_d$) controller gains.

An Upper Confidence Bound (UCB) active-learning acquisition function evaluates the confidence intervals on-chip to predict the next set of gains worth testing.

To overcome the $O(N^3)$ computational bottleneck of the repeated Cholesky decompositions the Gaussian Process relies on, the inner loop of the decomposition is being moved into handwritten ARM Thumb-2 assembly, targeting the Cortex-M7's FPU and VFMA (vector fused multiply-accumulate) instructions. The same numerical core compiles and runs on a laptop as well as on the Teensy, so the controller and optimizer can be validated in simulation before they ever touch hardware.

## Hardware Components

This project is built using a mix of standard prototyping gear and a couple of targeted upgrades made after the first hardware bring-up revealed where the cheap parts fell short:

* **Microcontroller:** Teensy 4.0 (NXP iMXRT1062 ARM Cortex-M7 at 600 MHz, 1MB RAM with 512KB tightly-coupled, 2MB Flash, pre-soldered headers for breadboard prototyping)
* **The Thermal Mass:** 40mm x 40mm x 20mm aluminum heatsink, bonded to the heater so the sensor and heater share the same body
* **The Heat Plant:** 10W 10-Ohm ceramic cement resistor
* **The Heater Switch:** IRLZ44N logic-level N-channel MOSFET. This replaced an L293D on the heater path. The L293D's bipolar output drops three to four volts internally, which starved the resistor and cooked the chip instead of the block, so the heater could never reach setpoint. The MOSFET's on-resistance is a few hundredths of an ohm, so almost all of the 9V reaches the resistor and the switch stays cool.
* **The Cooling Plant:** 5V DC cooling fan, still driven through the L293D since the fan draws little current and the chip handles it comfortably. Its PWM duty is capped in firmware so the 5V fan never sees the full 9V rail.
* **The Sensor:** 10k NTC thermistor (Beta 3950) read through a fixed-resistor voltage divider into the Teensy's ADC. A stainless steel probe version on a one-meter cable is used so the sensing tip clamps firmly to the heatsink instead of a loose bead drifting in and out of contact.
* **Power:** 9V battery feeding the heater rail through a breadboard-mounted barrel jack, with the Teensy powered over USB.

## Building and Running

The project uses PlatformIO, which downloads the Teensy toolchain and framework on the first build, so there is no separate dependency install step.

* **Firmware:** `pio run -e teensy40` builds the optimizer firmware, and `pio run -e teensy40 -t upload` flashes it.
* **Hardware bring-up:** `pio run -e hardware_check -t upload` flashes a separate menu-driven tester that exercises the LED, sensor, heater, and fan one at a time, so the wiring can be verified before trusting the full controller.
* **Simulation:** The `sim/` directory builds with plain `g++` and runs the entire optimizer against a lumped-capacitance thermal model on a laptop, no hardware required.
* **Unit tests:** The `test/` directory holds native checks for the matrix, GP, and controller math.

## Project Status

This repository is an active work in progress. Current state:

* The firmware, the laptop simulation, and the unit tests all build and pass.
* In simulation the optimizer converges, settling on consistent gains with sub-degree tracking error.
* The hardware is wired and validated end to end: the sensor reads real temperature, the heater warms the block, and the fan cools it, all confirmed through the bring-up tester.
* The heater driver and temperature sensor were upgraded after the first full hardware run showed the original L293D could not push the block to setpoint. Re-running the optimizer on the upgraded hardware is the next step.

Still ahead: replacing the Cholesky inner-loop assembly stub with a real hand-tuned VFMA implementation and benchmarking it against the C++ version, unifying the hardware and simulation trial-scoring protocols so their results can be compared directly, and a Python telemetry dashboard for live plotting of the GP posterior as the optimizer explores.