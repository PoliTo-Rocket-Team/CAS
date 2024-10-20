% Multiple Target Apogee Simulation with Delta RMSE for multiple algo
% resolutions
% This script runs the rocket simulation for various target apogees
% and prints/plots various informations regarding the performance of the
% predictive algo for varying degree of angle flap extension tolerance.

close all;
clear all;
clc;

% Simulation parameters
Ts = 0.02; % Sampling time (s)
simulationTime = 25; % Total simulation time (s), set to ensure rocket reaches apogee

% Target apogees range
apogee_min = 2000; % Minimum target apogee (m)
apogee_max = 3300; % Maximum target apogee (m)
apogee_step = 10;  % Step size for target apogee (m)
target_apogees = apogee_min:apogee_step:apogee_max; % Range of target apogees

% List of tolerance values to test
tol_values = [0.1 0.2 0.4 0.8 1.6 3.2 6.4 12.8 25.6];
% tol_values = linspace(0.1,30,100);

% Prepare arrays to store the RMSE and percentages for each tolerance value
RMSE_values = zeros(length(tol_values), 1);
percentage_successful_apogees = zeros(length(tol_values), 1); % To store percentage of successful apogees
successful_apogee_counts = zeros(length(tol_values), 1); % Counts for successful apogees

%% Rocket Simulation Parameters
params.g = 9.81; % Gravitational acceleration (m/s^2)
params.rho = 1.225; % Air density at sea level (kg/m^3)
params.Cd_f = 0.90; % Drag coefficient for flaps
params.Cd_R = 0.50; % Drag coefficient for the rocket body
params.A_f = 2e-3; % Reference area of flaps (m^2)
params.A_R = 1.41e-2; % Reference area of the rocket body (m^2)
params.n_flaps = 3; % Number of control flaps

% Loop over tolerance values
for tol_idx = 1:length(tol_values)
    tol = tol_values(tol_idx); % Current tolerance value
    
    % Prepare an array to store the delta between target and achieved apogee
    delta_apogees = zeros(length(target_apogees), 1);
    
    % Loop over different target apogees
    count_delta_less_than_5m = 0; % Initialize count for each tol
    achieved_apogees_within_5m = []; % Initialize array to store achieved apogees within 5m delta

    % Kalman Filter Parameters    
    F = [1 Ts Ts^2/2; 0 1 Ts; 0 0 1];  % State transition matrix
    B = [0; 0; Ts];  % Control input matrix
    H = [1 0 0; 0 0 1];
    
    for i = 1:length(target_apogees)
        target_apogee = target_apogees(i); % Current target apogee
        
        mass_initial = 24; % Initial mass of the rocket at launch (kg)
        mass_final = 17; % Final mass of the rocket after fuel burn (kg)
        [time_thrust, thrust] = thrustProfile('8088M1790-P.csv');
        burn_time_idx = find(thrust == 0 & time_thrust > 0, 1);
        burn_time = time_thrust(burn_time_idx);
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

            % Update if after KF start time
            if t > kf_start_time
                % Kalman Filter Update (keep only the selected one uncommented)
                % [x_est, P] = unscentedKalmanFilter(x_est, P, z, delta, Ts, Q, R, params, mass); % Unscented Kalman filter update
                [x_est, P] = kalmanFilter(x_est, P, z, a_m, F, B, H, Q, R); % Kalman filter update
                % [x_est, P] = extendedKalmanFilter(x_est, P, z, delta, Ts, Q, R, params, mass); % Extended Kalman filter update
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

        % Log the delta apogee for RMSE calculation later
        delta_apogees(i) = delta_apogee;
        
        % Count the number of apogees with delta < 5m
        if abs(delta_apogee) < 5
            count_delta_less_than_5m = count_delta_less_than_5m + 1;
            achieved_apogees_within_5m = [achieved_apogees_within_5m; final_apogee]; % Store achieved apogee
        end
    end
    
    % Filter delta_apogees to only those with delta < 5m for RMSE calculation
    valid_deltas = delta_apogees(abs(delta_apogees) < 5);
    
    % Calculate RMSE for this tolerance value if there are valid deltas
    if ~isempty(valid_deltas)
        RMSE = sqrt(mean(valid_deltas.^2));
    else
        RMSE = NaN; % Set RMSE to NaN if there are no valid deltas
    end
    
    % Store the RMSE value
    RMSE_values(tol_idx) = RMSE;
    
    % Store the count of successful apogees for this tolerance
    successful_apogee_counts(tol_idx) = count_delta_less_than_5m;
    
    % Calculate and store the percentage of successful apogees for this tolerance
    if successful_apogee_counts(1) > 0 % Use first tolerance count as reference
        percentage_successful_apogees(tol_idx) = (count_delta_less_than_5m / successful_apogee_counts(1)) * 100; % Percentage
    else
        percentage_successful_apogees(tol_idx) = 0; % Avoid division by zero
    end

    % Find lowest and highest achieved apogee within 5m delta
    if ~isempty(achieved_apogees_within_5m)
        lowest_apogee = min(achieved_apogees_within_5m);
        highest_apogee = max(achieved_apogees_within_5m);
    else
        lowest_apogee = NaN;
        highest_apogee = NaN;
    end

    % Print the result in the console in a single line
    fprintf('Tol: %.1f°, RMSE: %.4f m, %s Delta < 5m: %.2f%%, Lowest Apogee: %.2f m, Highest Apogee: %.2f m\n', ...
            tol, RMSE, 'Percentage of Apogees', percentage_successful_apogees(tol_idx), lowest_apogee, highest_apogee);
end

%% Plotting RMSE vs. Tolerance
figure;
subplot(2, 1, 1);
plot(tol_values, RMSE_values, '-', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('Tolerance (degrees)');
ylabel('RMSE (m)');
title('RMSE of Delta Apogee (<5m Delta) vs. Tolerance');
xlim([tol_values(1) tol_values(end)]);
grid on;

%% Plotting Bar Graph of Percentage of Successful Apogees (Delta < 5m) vs. Tolerance
subplot(2, 1, 2);
plot(tol_values, percentage_successful_apogees, '-r', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('Tolerance (degrees)');
ylabel('% of Successful Apogees');
title('Percentage of Apogees with Delta < 5m vs. Tolerance');
xlim([tol_values(1) tol_values(end)]);
grid on;
