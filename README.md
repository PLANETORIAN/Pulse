# High-Resolution Pulsed Doppler Radar DSP & Detection Pipeline

![MATLAB](https://img.shields.io/badge/MATLAB-Data_Processing-blue.svg) ![Simulink](https://img.shields.io/badge/Simulink-DSP_Modeling-blue.svg) ![Hardware Ready](https://img.shields.io/badge/Status-Hardware_Ready-success.svg)

## Project Overview
This repository contains a defense-grade Pulsed Doppler Radar simulation modeled entirely in MATLAB and Simulink. The system takes raw RF physics (electromagnetic echoes, thermal noise, and target fluctuations) and processes them through a complete Digital Signal Processing (DSP) pipeline to extract precise target range and velocity data. 

The architecture is explicitly designed for eventual fixed-point conversion and digital ASIC synthesis, utilizing a 16-pulse Coherent Processing Interval (CPI) mapped into a stable 1000x16 Range-Doppler memory matrix. 

## System Architecture

### 1. RF Physics & Front-End Model
*   **Operating Parameters:** 10 GHz carrier frequency, 100 kW transmit power.
*   **Target Kinematics:** Baseband receiver voltage calculated dynamically using the Radar Range Equation.
*   **Statistical Fading:** Implements native **Swerling I target fluctuation models** using inverse transform sampling, simulating chaotic radar cross-section (RCS) aspect changes and Rayleigh-distributed voltage envelopes.

### 2. Fast-Time Processing (Range Compression)
*   **Matched Filtering:** Discrete FIR filter blocks cross-correlate incoming echoes to compress raw rectangular pulses into discrete wave peaks.
*   **DSP Windowing:** Hamming coefficients are applied directly to the Matched Filter to smooth main lobes and drastically suppress transient high-frequency ripples.

### 3. Slow-Time Processing (Doppler Extraction)
*   **Matrix Buffering:** Sequential 1D hardware data is streamed and buffered into a 1000x16 2D matrix.
*   **Spectral Analysis:** A 16-point FFT analyzes pulse-to-pulse phase rotation to extract target velocity (Doppler shift).
*   **Leakage Control:** Broadcasted Hamming windowing crushes Doppler sidelobes to prevent target self-masking.

### 4. Autonomous Detection
*   **2D CA-CFAR:** A Cell-Averaging Constant False Alarm Rate sliding mathematical mask dynamically calculates the thermal noise floor across the Range-Doppler matrix.
*   **Self-Masking Prevention:** Custom-calibrated 2D Guard Cells (Gr=100, Gd=1) accommodate the physical spread of the Hamming-windowed pulses.

## Statistical Validation (Monte Carlo)
To prove the structural integrity of the detection algorithm under real-world physical stress, the system includes an automated Monte Carlo validation suite.
*   **Probability of False Alarm Validation:** Ensures false alarm rates hold strictly to the designed threshold in noise-only environments.
*   **Probability of Detection Validation:** Automated trials run the Simulink model while bombarding the target with randomized Swerling I RCS fluctuations. The current DSP pipeline successfully maintains a **99.00% Probability of Detection** against severe statistical fading.

## Repository Structure
*   `radar_init.m`: Master initialization script. Pre-calculates static RF physics, constants, and multi-target spatial parameters before execution.
*   `Pulse_Radar.slx`: The core Simulink DSP and RF physics model. 
*   `run_monte_carlo.m`: Automated validation script that injects randomized physical seeds into the Simulink model to test threshold resilience.

## Current Project Phase & Roadmap

- [x] **Phases 1-4:** RF Physics, Fast-Time Range Compression, Slow-Time FFT, and baseline 2D CFAR.
- [x] **Phase 8.5:** Swerling I statistical validation and Monte Carlo testing.
- [ ] **Phase 8.6:** Multi-target spatial distributed field testing (Resolution cell validation for closely spaced objects).
- [ ] **Phase 9:** Track Manager & Centroiding (Computer vision segmentation to extract `[X, Y]` coordinates from the binary CFAR matrix).
- [ ] **Phase 10:** Fixed-Point Conversion (Simulink Fixed-Point Designer to quantify quantization loss in the FFT/CFAR stages).
- [ ] **Phase 11:** Verilog RTL Generation & Digital Logic Synthesis via HDL Coder.

## Usage
1. Open MATLAB and navigate to the repository directory.
2. Run `radar_init.m` to load the baseline physical constants and spatial targets into the workspace.
3. Open `Pulse_Radar.slx` to view the DSP pipeline or run the simulation manually.
4. (Optional) Run `run_monte_carlo.m` to execute the automated 100-trial statistical validation suite.
