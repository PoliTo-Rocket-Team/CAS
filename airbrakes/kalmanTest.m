% Multiple Target Apogee Simulation with Delta RMSE for multiple Kalmans
% This script runs the rocket simulation for various target apogees
% and plots the delta (difference) between the target and achieved apogee.

close all;
clear all;
clc;


% Target apogees range
apogee_min = 2000; % Minimum target apogee (m)
apogee_max = 3300; % Maximum target apogee (m)
apogee_step = 10;  % Step size for target apogee (m)
target_apogees = apogee_min:apogee_step:apogee_max; % Range of target apogees

% Prepare arrays to store the delta between target and achieved apogee for each filter
delta_apogees_KF = zeros(length(target_apogees), 1);
delta_apogees_EKF = zeros(length(target_apogees), 1);
delta_apogees_UKF = zeros(length(target_apogees), 1);

%% Rocket Simulation Parameters
params.g = 9.81; % Gravitational acceleration (m/s^2)
params.rho = 1.225; % Air density at sea level (kg/m^3)
params.Cd_f = 0.90; % Drag coefficient for flaps
params.Cd_R = 0.50; % Drag coefficient for the rocket body
params.A_f = 2e-3; % Reference area of flaps (m^2)
params.A_R = 1.41e-2; % Reference area of the rocket body (m^2)
params.n_flaps = 3; % Number of control flaps

tol = 0.1; % tolerance for the binary search algo (degrees)

%% Kalman Filter (KF) Simulation Loop
fprintf('Running KF Simulation...\n');
rng('default')
for i = 1:length(target_apogees)
    target_apogee = target_apogees(i); % Current target apogee
       
    mass_initial = 24; % Initial mass of the rocket at launch (kg)
    mass_final = 17; % Final mass of the rocket after fuel burn (kg)
    [time_thrust, thrust] = thrustProfile('8088M1790-P.csv');
    burn_time_idx = find(thrust == 0 & time_thrust > 0, 1);
    burn_time = time_thrust(burn_time_idx);
    Ts = 0.02; % Sampling time (s)
    simulationTime = 25; % Total simulation time (s)
    max_delta_rate = deg2rad(45);
    activation_time = burn_time + 1; % Time (s) when airbrakes can start working
    kf_start_time = burn_time + 0.1;
    
    % Kalman Filter Parameters
    F = [1 Ts Ts^2/2; 0 1 Ts; 0 0 1];  % State transition matrix
    B = [0; 0; Ts];  % Control input matrix
    H = [1 0 0; 0 0 1];

    % Process and measurement noise covariance matrices
    Q = [1e-5 0 0; 0 1e-3 0; 0 0 5e-3];
    sigma_h = 0.5; % Measurement noise standard deviation for height
    sigma_a = 0.1; % Measurement noise standard deviation for acceleration
    R = [sigma_h^2 0; 0 sigma_a^2];  % Measurement noise covariance matrix
    
    % Initial state and covariance matrix
    x_est = [750; 300; -35]; % Initial state estimate [h, v, a]
    P = eye(3); % Initial state covariance matrix
    
    % Simulation initial conditions
    h_actual = 0;
    v_actual = 0;
    a_actual = 0;
    delta = 0;
    mass = mass_initial;
    actualHeightLog = zeros(simulationTime/Ts, 1);
    
    % Simulation Loop
    for k = 1:simulationTime/Ts
        t = (k-1) * Ts;
        thrust_current = interp1(time_thrust, thrust, t, 'linear', 0);
        mass = rocketMass(t, burn_time, mass_initial, mass_final); % Update mass
        drag_force = 0.5 * params.rho * (params.n_flaps * params.Cd_f * params.A_f * sin(delta) + params.Cd_R * params.A_R) * v_actual^2;
        a_actual = -drag_force / mass - params.g + thrust_current / mass;
        

        % Noisy measurements
        h_m = h_actual + randn * sqrt(R(1,1));
        a_m = a_actual + randn * sqrt(R(2,2));
        z = [h_m; a_m]; % Measurement vector (altitude, acceleration)
        
        % Process noise addition
        process_noise = [randn * sqrt(Q(1,1)); randn * sqrt(Q(2,2)); randn * sqrt(Q(3,3))];
        a_actual = a_actual + process_noise(3); % Add process noise to acceleration
        
        % Update if after KF start time
        if t > kf_start_time
            [x_est, P] = kalmanFilter(x_est, P, z, a_m, F, B, H, Q, R); % Kalman filter update
        end
        
        actualHeightLog(k) = h_actual; % Log actual height
        
        % Airbrake control logic
        if t > activation_time
            optimal_delta = findOptimalFlapAngle(x_est(1), x_est(2), target_apogee, mass, params.Cd_f, params.A_f, params.Cd_R, params.A_R, params.g, params.rho, Ts, params.n_flaps, delta, max_delta_rate, tol);
            delta_change = optimal_delta - delta;
            max_change = max_delta_rate * Ts;
            if abs(delta_change) > max_change
                delta = delta + sign(delta_change) * max_change;
            else
                delta = optimal_delta;
            end
        end
        
        % Update velocity and height based on actual acceleration
        v_actual = v_actual + (a_actual * Ts);
        h_actual = h_actual + (v_actual * Ts);
    end
    
    % Apogee calculation for this run
    final_apogee = max(actualHeightLog);
    delta_apogee = final_apogee - target_apogee; % Delta between achieved and target apogee
    
    % Log the delta apogee for plotting later
    delta_apogees_KF(i) = delta_apogee;
    
end

%% Extended Kalman Filter (EKF) Simulation Loop
fprintf('Running EKF Simulation...\n');
rng('default')
for i = 1:length(target_apogees)
    target_apogee = target_apogees(i); % Current target apogee
       
    mass_initial = 24; % Initial mass of the rocket at launch (kg)
    mass_final = 17; % Final mass of the rocket after fuel burn (kg)
    [time_thrust, thrust] = thrustProfile('8088M1790-P.csv');
    burn_time_idx = find(thrust == 0 & time_thrust > 0, 1);
    burn_time = time_thrust(burn_time_idx);
    Ts = 0.02; % Sampling time (s)
    simulationTime = 25; % Total simulation time (s)
    max_delta_rate = deg2rad(45);
    activation_time = burn_time + 1; % Time (s) when airbrakes can start working
    kf_start_time = burn_time + 0.1;
    
    % Process and measurement noise covariance matrices
    Q = [1e-5 0 0; 0 1e-3 0; 0 0 5e-3];
    sigma_h = 0.5; % Measurement noise standard deviation for height
    sigma_a = 0.1; % Measurement noise standard deviation for acceleration
    R = [sigma_h^2 0; 0 sigma_a^2];  % Measurement noise covariance matrix
    
    % Initial state and covariance matrix
    x_est = [750; 300; -35]; % Initial state estimate [h, v, a]
    P = eye(3); % Initial state covariance matrix
    
    % Simulation initial conditions
    h_actual = 0;
    v_actual = 0;
    a_actual = 0;
    delta = 0;
    mass = mass_initial;
    actualHeightLog = zeros(simulationTime/Ts, 1);
    
    % Simulation Loop
    for k = 1:simulationTime/Ts
        t = (k-1) * Ts;
        thrust_current = interp1(time_thrust, thrust, t, 'linear', 0);
        mass = rocketMass(t, burn_time, mass_initial, mass_final); % Update mass
        drag_force = 0.5 * params.rho * (params.n_flaps * params.Cd_f * params.A_f * sin(delta) + params.Cd_R * params.A_R) * v_actual^2;
        a_actual = -drag_force / mass - params.g + thrust_current / mass;
        

        % Noisy measurements
        h_m = h_actual + randn * sqrt(R(1,1));
        a_m = a_actual + randn * sqrt(R(2,2));
        z = [h_m; a_m]; % Measurement vector (altitude, acceleration)
        
        % Process noise addition
        process_noise = [randn * sqrt(Q(1,1)); randn * sqrt(Q(2,2)); randn * sqrt(Q(3,3))];
        a_actual = a_actual + process_noise(3); % Add process noise to acceleration
        
        % Update if after EKF start time
        if t > kf_start_time
            [x_est, P] = extendedKalmanFilter(x_est, P, z, delta, Ts, Q, R, params, mass); % Extended Kalman filter update
        end
        
        actualHeightLog(k) = h_actual; % Log actual height
        
        % Airbrake control logic
        if t > activation_time
            optimal_delta = findOptimalFlapAngle(x_est(1), x_est(2), target_apogee, mass, params.Cd_f, params.A_f, params.Cd_R, params.A_R, params.g, params.rho, Ts, params.n_flaps, delta, max_delta_rate, tol);
            delta_change = optimal_delta - delta;
            max_change = max_delta_rate * Ts;
            if abs(delta_change) > max_change
                delta = delta + sign(delta_change) * max_change;
            else
                delta = optimal_delta;
            end
        end
        
        % Update velocity and height based on actual acceleration
        v_actual = v_actual + (a_actual * Ts);
        h_actual = h_actual + (v_actual * Ts);
    end
    
    % Apogee calculation for this run
    final_apogee = max(actualHeightLog);
    delta_apogee = final_apogee - target_apogee; % Delta between achieved and target apogee
    
    % Log the delta apogee for plotting later
    delta_apogees_EKF(i) = delta_apogee;
    
end

%% Unscented Kalman Filter (UKF) Simulation Loop
fprintf('Running UKF Simulation...\n');
rng('default')
for i = 1:length(target_apogees)
    target_apogee = target_apogees(i); % Current target apogee
       
    mass_initial = 24; % Initial mass of the rocket at launch (kg)
    mass_final = 17; % Final mass of the rocket after fuel burn (kg)
    [time_thrust, thrust] = thrustProfile('8088M1790-P.csv');
    burn_time_idx = find(thrust == 0 & time_thrust > 0, 1);
    burn_time = time_thrust(burn_time_idx);
    Ts = 0.02; % Sampling time (s)
    simulationTime = 25; % Total simulation time (s)
    max_delta_rate = deg2rad(45);
    activation_time = burn_time + 1; % Time (s) when airbrakes can start working
    kf_start_time = burn_time + 0.1;
    
    % Process and measurement noise covariance matrices
    Q = [1e-5 0 0; 0 1e-3 0; 0 0 5e-3];
    sigma_h = 0.5; % Measurement noise standard deviation for height
    sigma_a = 0.1; % Measurement noise standard deviation for acceleration
    R = [sigma_h^2 0; 0 sigma_a^2];  % Measurement noise covariance matrix
    
    % Initial state and covariance matrix
    x_est = [750; 300; -35]; % Initial state estimate [h, v, a]
    P = eye(3); % Initial state covariance matrix
    
    % Simulation initial conditions
    h_actual = 0;
    v_actual = 0;
    a_actual = 0;
    delta = 0;
    mass = mass_initial;
    actualHeightLog = zeros(simulationTime/Ts, 1);
    
    % Simulation Loop
    for k = 1:simulationTime/Ts
        t = (k-1) * Ts;
        thrust_current = interp1(time_thrust, thrust, t, 'linear', 0);
        mass = rocketMass(t, burn_time, mass_initial, mass_final); % Update mass
        drag_force = 0.5 * params.rho * (params.n_flaps * params.Cd_f * params.A_f * sin(delta) + params.Cd_R * params.A_R) * v_actual^2;
        a_actual = -drag_force / mass - params.g + thrust_current / mass;
        

        % Noisy measurements
        h_m = h_actual + randn * sqrt(R(1,1));
        a_m = a_actual + randn * sqrt(R(2,2));
        z = [h_m; a_m]; % Measurement vector (altitude, acceleration)
        
        % Process noise addition
        process_noise = [randn * sqrt(Q(1,1)); randn * sqrt(Q(2,2)); randn * sqrt(Q(3,3))];
        a_actual = a_actual + process_noise(3); % Add process noise to acceleration
        
        % Update if after UKF start time
        if t > kf_start_time
            [x_est, P] = unscentedKalmanFilter(x_est, P, z, delta, Ts, Q, R, params, mass); % Unscented Kalman filter update
        end
        
        actualHeightLog(k) = h_actual; % Log actual height
        
        % Airbrake control logic
        if t > activation_time
            optimal_delta = findOptimalFlapAngle(x_est(1), x_est(2), target_apogee, mass, params.Cd_f, params.A_f, params.Cd_R, params.A_R, params.g, params.rho, Ts, params.n_flaps, delta, max_delta_rate, tol);
            delta_change = optimal_delta - delta;
            max_change = max_delta_rate * Ts;
            if abs(delta_change) > max_change
                delta = delta + sign(delta_change) * max_change;
            else
                delta = optimal_delta;
            end
        end
        
        % Update velocity and height based on actual acceleration
        v_actual = v_actual + (a_actual * Ts);
        h_actual = h_actual + (v_actual * Ts);
    end
    
    % Apogee calculation for this run
    final_apogee = max(actualHeightLog);
    delta_apogee = final_apogee - target_apogee; % Delta between achieved and target apogee
    
    % Log the delta apogee for plotting later
    delta_apogees_UKF(i) = delta_apogee;
    
end

%% Calculate and Display RMSE for Each Filter
rmse_KF = sqrt(mean(delta_apogees_KF.^2));
rmse_EKF = sqrt(mean(delta_apogees_EKF.^2));
rmse_UKF = sqrt(mean(delta_apogees_UKF.^2));

% Display the RMSE values in the command line
fprintf('RMSE for KF: %.2f m\n', rmse_KF);
fprintf('RMSE for EKF: %.2f m\n', rmse_EKF);
fprintf('RMSE for UKF: %.2f m\n', rmse_UKF);
