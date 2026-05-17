# Autonomous Cyber-Physical Thermal Optimizer (ACPTO)

The Autonomous Cyber-Physical Thermal Optimizer (ACPTO) is an embedded, edge-computing research platform designed to explore the intersection of machine learning, low-level computer architecture, and classical control theory. 

The goal of this project is to build an asymmetric, dual-core hardware-in-the-loop testbed that can autonomously map out and tune its own physical thermodynamic control systems in real time without human intervention.

## Why I am Building This
Deep-space exploration payloads and advanced laboratory instruments face volatile environments where thermal drift can quickly compromise sensitive sensors. Traditional architectures rely on static control parameters that cannot adapt to hardware degradation or unmodeled atmospheric shocks. Furthermore, running heavy statistical optimization models typically requires high-power desktop infrastructure.

This project serves as a proof-of-concept for moving complex statistical inference directly onto low-resource edge hardware. By bypassing some libraries and optimizing the core mathematical bottlenecks at the register level, I aim to have a system that is robust with self-healing autonomy that is achieved on localized silicon.

## System Architecture & Theory

The project utilizes a dual-core architectural split on an ESP32-S3 microcontroller to achieve strict isolation of concerns:
* **Core 0 (The Deterministic Control Plant):** Executes a high-frequency, time-critical PID loop to actively manage the physical temperature of an aluminum mass.
* **Core 1 (The Mathematical Coprocessor):** Runs a localized 2D Gaussian Process (GP) regression with a Radial Basis Function (RBF) kernel to model the performance landscape across proportional ($K_p$) and derivative ($K_d$) controller gains.

An Upper Confidence Bound (UCB) active-learning acquisition function evaluates the confidence intervals on-chip to predict the next optimal parameters to test. 

To overcome the $O(N^3)$ computational bottleneck of continuous matrix inversions required by the Gaussian Process, the inner loops of the Cholesky matrix decomposition framework are optimized directly using handwritten Xtensa LX7 assembly instructions and specialized vector extensions.

## Hardware Components
This project is built using a mix of standard prototyping gear and specialized high-performance edge hardware:
* **Microcontroller:** ESP32-S3 DevKitC-1 (N16R8 variant with 16MB Flash and 8MB PSRAM)
* **The Thermal Mass:** 40mm x 40mm x 20mm Anodized Aluminum Heatsink
* **The Heat Plant:** BOJACK 10W 10-Ohm Ceramic Cement Resistor
* **The Cooling Plant:** 5V Brushless DC Cooling Fan
* **The Gatekeeper:** L293D H-Bridge Integrated Circuit
* **The Sensor:** 10k NTC Thermistor paired with a 10k Ohm fixed resistor (Voltage Divider configuration)

## Project Status
**Note:** This repository is currently a work in progress. Early phases focusing on hardware calibration and baseline C++ matrix classes are underway. Custom assembly kernels and the wireless Python-based data telemetry dashboard will be uploaded as development progresses throughout the summer.
