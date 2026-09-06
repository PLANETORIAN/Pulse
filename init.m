    % radar_init.m
    c = 3e8;                    
    PRI = 1e-3;                 
    tau_p = 0.1e-3;             
    R = 20000;                  
    tau = (2 * R) / c;                       
    Ts = 1e-6;                  % 1 microsecond sample time (150m range bins)
    R_min = (c*tau_p)/2;
    
    % --- Discrete DSP Parameters for Simulink ---
    N_PRI = round(PRI / Ts);      % Period in samples (1000)
    N_tau_p = round(tau_p / Ts);  % Pulse width in samples (100)
    N_delay = round(tau / Ts);    % Round-trip delay in samples (100)
    noise_variance = 0.64;        % Variance of thermal noise (sigma = 0.8)
    
    
    % --- RF & Physical Parameters ---
    fc = 10e9;                  % Carrier frequency (10 GHz)
    lambda = c / fc;            % Wavelength (meters)
    Pt = 100e3;                 % Peak transmit power (100 kW)
    G_dB = 35;                  % Antenna gain (dB)
    G = 10^(G_dB / 10);         % Antenna gain (linear)
    sigma_rcs = 1;              % Target RCS (1 square meter - e.g., small jet)

    % --- Phase 8.5: Swerling I Target Model ---
   sigma_avg = 1;                     % Average RCS in square meters
   
   % Create a default seed so Simulink can run manually from the GUI
   current_seed = randi(100000);
   
   % We pre-calculate all the static physics variables into one constant
   radar_const = (Pt * G^2 * lambda^2) / (((4*pi)^3) * (R^4) * L);
   
   Loss_dB = 2;                % System losses (dB)
    L = 10^(Loss_dB / 10);      % System losses (linear)
    
    % --- Target Amplitude (Radar Range Equation) ---
    % Pr = (Pt * G^2 * lambda^2 * sigma_rcs) / ((4*pi)^3 * R^4 * L)
    Pr = (Pt * G^2 * lambda^2 * sigma_rcs) / (((4*pi)^3) * (R^4) * L);
    V_rx = sqrt(Pr);            % Baseband voltage amplitude
    
    % --- Receiver Thermal Noise ---
    k_B = 1.38e-23;             % Boltzmann constant (J/K)
    T0 = 290;                   % Standard temperature (K)
    B = 1 / Ts;                 % Receiver bandwidth (1 MHz, based on Ts)
    NF_dB = 3;                  % Receiver Noise Figure (dB)
    F = 10^(NF_dB / 10);        % Noise Figure (linear)
    
    Pn = k_B * T0 * B * F;      % Noise power (Variance for AWGN)
    noise_variance_IQ = Pn / 2;
    
    
    
    % --- CA-CFAR Parameters ---
    Pfa = 1e-6;                 % Desired Probability of False Alarm (1 in a million)
    N_train = 32;               % Number of training cells (16 leading, 16 lagging)
    N_guard = 200;              % Guard cells (100 per side to hide the 200-sample triangle base)
    
    % 1. Calculate the Threshold Multiplier (alpha) for Square-Law Detection
    alpha = N_train * (Pfa^(-1/N_train) - 1);
    
    % 2. Create the HDL-Portable Sliding Window (FIR Taps)
    % [16 Ones] + [201 Zeros (Guard + CUT)] + [16 Ones] ... divided by N_train for average
    cfar_window = [ones(1, N_train/2), zeros(1, N_guard + 1), ones(1, N_train/2)] / N_train;
    
    % 3. Calculate the delay to align the CUT with the Threshold
    cfar_delay = (N_train/2) + (N_guard/2);

    % --- Target Kinematics & Doppler ---
    target_velocity = 200;                  % Target moving at 200 m/s (approx Mach 0.6)
    fd = (2 * target_velocity) / lambda;    % Doppler frequency shift (Hz)

    % --- Coherent Processing Interval (CPI) ---
    N_cpi = 16;                           % Pulses per CPI (FFT size for Doppler)
    T_stop = N_cpi * PRI;                 % Extend simulation to exactly 1 CPI

    % --- MTI Canceller Parameters ---
    % A 2-pulse canceller notch filter is simply: H(z) = 1 - z^(-N_PRI)
    MTI_delay = N_PRI;                    % Delay one full PRI (1000 samples)

    % --- Range Calculation Parameters ---
    % Distance = (Speed of Light * Time) / 2 (for round trip)
    range_per_sample = (c * Ts) / 2;    % Physical distance per sample bin (meters)

   % --- Hardware Calibration (Pipeline Latency) ---
matched_filter_delay = N_tau_p - 1;       % Peak occurs at N-1 in discrete convolution
drop_threshold_delay = 8;
hardware_delay_samples = matched_filter_delay + cfar_delay + drop_threshold_delay; 
% Total delay is now 223

% --- Maximum Velocity calculated ---
V_max = lambda/(4*Ts*N_PRI);

% --- DSP Windowing Functions ---
doppler_window = hamming(N_cpi)';      % 1x16 curve to crush FFT leakage
range_window = hamming(N_tau_p)';      % 1x100 curve to smooth the matched filter