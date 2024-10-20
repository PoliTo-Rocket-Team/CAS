%% Rocket Simulation Script
% This script simulates the flight of a rocket using a Kalman filter for state
% estimation and binary search for optimal flap angle control to achieve the
% desired apogee.

close all;
clear all;
clc;

%% Parameters
g = 9.81; % Gravitational acceleration (m/s^2)
rho = 1.225; % Air density at sea level (kg/m^3)

% Rocket mass parameters
mass_initial = 24; % Initial mass of the rocket at launch (kg)
mass_final = 17; % Final mass of the rocket after fuel burn (kg)

% Aerodynamic drag parameters
Cd_f = 0.90; % Drag coefficient for flaps
Cd_R = 0.50; % Drag coefficient for the rocket body
A_f = 2e-3; % Reference area of flaps (m^2)
A_R = 1.41e-2; % Reference area of the rocket body (m^2)
n_flaps = 3; % Number of control flaps

% Thrust profile (time vs thrust in N), read from external file
[time_thrust, thrust] = thrustProfile('8088M1790-P.csv');

% Find the burn time (first time when thrust drops to 0 after launch)
burn_time_idx = find(thrust == 0 & time_thrust > 0, 1); % Index of the first zero-thrust point after time = 0
burn_time = time_thrust(burn_time_idx); % Burn time in seconds

% Simulation parameters
Ts = 0.02; % Sampling time (s)
simulationTime = 25; % Total simulation time (s), set to ensure rocket reaches apogee

% Rate constraints and activation time for flap deployment
max_delta_rate = deg2rad(45); % Maximum flap angle change rate (in degrees/second, converted to radians)
activation_time = burn_time + 1; % Time (s) when airbrakes can start working (1 second after burn-off)

% Define the Kalman filter start time
kf_start_time = burn_time + 0.1;  % Kalman filter starts 0.1 seconds after burnout

% Kalman Filter Parameters
F = [1 Ts Ts^2/2; 0 1 Ts; 0 0 1];  % State transition matrix
B = [0; 0; Ts];  % Control input matrix
H = [1 0 0; 0 0 1];


% Process and measurement noise covariance matrices
Q = [1e-5 0 0; 0 1e-3 0; 0 0 5e-3];  % Process noise covariance matrix
sigma_h = 0.5;  % Standard deviation of height measurement noise (m)
sigma_a = 0.1;  % Standard deviation of acceleration measurement noise (m/s^2)
R = [sigma_h^2 0; 0 sigma_a^2];  % Measurement noise covariance matrix

% Initial state estimate and covariance matrix
x_est = [750; 300; -35];  % Initial state [altitude, velocity, acceleration] (m, m/s, m/s^2)
P = eye(3); % Initial covariance matrix for state estimate uncertainty

%% Simulation Setup
h_actual = 0; % Initial actual height (m)
v_actual = 0; % Initial actual velocity (m/s)
a_actual = 0; % Initial actual acceleration (m/s^2) 
target_apogee = 2900; % Desired apogee (maximum height) in meters
delta = 0; % Initial flap angle (radians)
mass = mass_initial; % Initial rocket mass (kg)
tol = 0.1; % Tolerance for the binary search algorithm (degrees)

% Data logging arrays to store simulation results
heightLog = zeros(simulationTime/Ts, 1);      % Log for estimated height
velocityLog = zeros(simulationTime/Ts, 1);    % Log for estimated velocity
accelerationLog = zeros(simulationTime/Ts, 1); % Log for estimated acceleration
actualHeightLog = zeros(simulationTime/Ts, 1); % Log for actual height
actualVelocityLog = zeros(simulationTime/Ts, 1); % Log for actual velocity
actualAccelerationLog = zeros(simulationTime/Ts, 1); % Log for actual acceleration
flapLog = zeros(simulationTime/Ts, 1);       % Log for flap angle
thrustLog = zeros(simulationTime/Ts, 1);     % Log for thrust
massLog = zeros(simulationTime/Ts, 1);       % Log for mass

%% Bundle parameters into a struct for passing to functions
params.g = g;
params.rho = rho;
params.mass_initial = mass_initial;
params.mass_final = mass_final;
params.Cd_f = Cd_f;
params.Cd_R = Cd_R;
params.A_f = A_f;
params.A_R = A_R;
params.n_flaps = n_flaps;

%% Simulation Loop
for k = 1:simulationTime/Ts
    % Current time (s)
    t = (k-1) * Ts; % Calculate the current time based on iteration and sampling time
    
    % Thrust Calculation
    thrust_current = interp1(time_thrust, thrust, t, 'linear', 0); % Interpolate thrust at time t (N)
    thrustLog(k) = thrust_current; % Log the thrust for analysis
    
    % Mass Calculation
    mass = rocketMass(t, burn_time, mass_initial, mass_final); % Calculate current rocket mass (kg)
    massLog(k) = mass; % Log mass for analysis
    
    % Acceleration Calculation (Drag and Gravity)
    drag_force = 0.5 * rho * (n_flaps * Cd_f * A_f * sin(delta) + Cd_R * A_R) * v_actual^2; % Calculate drag force
    a_actual = -drag_force / mass - g; % Calculate acceleration due to drag and gravity (m/s^2)
    a_actual = a_actual + thrust_current / mass; % Include thrust in acceleration (m/s^2)

    % Measurement Noise Simulation
    h_m = h_actual + randn * sqrt(R(1,1)); % Simulate noisy height measurement
    a_m = a_actual + randn * sqrt(R(2,2)); % Simulate noisy acceleration measurement
    z = [h_m; a_m]; % Measurement vector combining height and acceleration

    % Process Noise Addition
    process_noise = [randn * sqrt(Q(1,1)); randn * sqrt(Q(2,2)); randn * sqrt(Q(3,3))]; % Generate process noise
    a_actual = a_actual + process_noise(3); % Apply noise to acceleration (affects velocity)
    
    % Activate Kalman Filter after the start time
    if t > kf_start_time
        % Kalman Filter Update
            % Kalman Filter Update (keep only the selected one uncommented)
            % [x_est, P] = unscentedKalmanFilter(x_est, P, z, delta, Ts, Q, R, params, mass); % Unscented Kalman filter update
            [x_est, P] = kalmanFilter(x_est, P, z, a_m, F, B, H, Q, R); % Kalman filter update
            % [x_est, P] = extendedKalmanFilter(x_est, P, z, delta, Ts, Q, R, params, mass); % Extended Kalman filter update
        
        % Log the estimated state
        heightLog(k) = x_est(1);        % Log estimated height (m)
        velocityLog(k) = x_est(2);      % Log estimated velocity (m/s)
        accelerationLog(k) = x_est(3);  % Log estimated acceleration (m/s^2)
    else
        % Before Kalman filter activation, log actual values for height and velocity
        heightLog(k) = NaN;        % No estimated height until Kalman starts
        velocityLog(k) = NaN;      % No estimated velocity until Kalman starts
        accelerationLog(k) = NaN;  % No estimated acceleration until Kalman starts
    end

    % Log the actual rocket state (actual height, velocity, and acceleration)
    actualHeightLog(k) = h_actual;      % Log actual height (m)
    actualVelocityLog(k) = v_actual;    % Log actual velocity (m/s)
    actualAccelerationLog(k) = a_actual; % Log actual acceleration (m/s^2)

    % Flap Angle Control (independent of Kalman filter)
    if t > activation_time
        % Find the optimal flap angle using binary search, considering the rate limit
        optimal_delta = findOptimalFlapAngle(x_est(1), x_est(2), target_apogee, mass, Cd_f, A_f, Cd_R, A_R, g, rho, Ts, n_flaps, delta, max_delta_rate, tol);
    
        % Apply Rate Constraint to Flap Angle Adjustment
        delta_change = optimal_delta - delta; % Calculate desired change in flap angle (radians)
        max_change = max_delta_rate * Ts; % Calculate maximum allowed change in this timestep (radians)
        
        % Apply the rate constraint
        if abs(delta_change) > max_change
            delta = delta + sign(delta_change) * max_change; % Increment delta by the allowed change
        else
            delta = optimal_delta; % Apply full change if within the allowed rate
        end
    end
    
    % Update Rocket State (Velocity and Height)
    v_actual = v_actual + (a_actual * Ts); % Update actual velocity (m/s)
    h_actual = h_actual + (v_actual * Ts); % Update actual height (m)

    % Apply constraints to prevent unrealistic states
    h_actual = max(h_actual, 0); % Prevent height from going negative
    if h_actual == 0
        v_actual = 0; % Reset velocity if the rocket has landed
    end

    % Log the current flap angle
    flapLog(k) = delta; % Log the flap angle (radians)
end


%% Apogee Calculation
final_apogee = max(actualHeightLog); % Find maximum height from the logged data
delta_apogee = final_apogee - target_apogee; % Calculate the difference from the target apogee

% Find the time at which the apogee is reached
apogee_time_idx = find(actualHeightLog == final_apogee, 1); % Index of the apogee
apogee_time = (apogee_time_idx - 1) * Ts; % Time at which the apogee is reached

% Print the apogee and delta in the console
fprintf('Final Apogee: %.2f meters\n', final_apogee);
fprintf('Delta from Target Apogee: %.2f meters\n', delta_apogee);
fprintf('Time to Apogee: %.2f seconds\n', apogee_time);

%% Plot the Results
% Figure 1: Altitude, Velocity, and Acceleration (Actual vs Estimated)
figure;
subplot(3,1,1);
plot(0:Ts:(simulationTime-Ts), heightLog, 'b', 'LineWidth', 1.5); hold on;
plot(0:Ts:(simulationTime-Ts), actualHeightLog, 'r--', 'LineWidth', 1.5);
xline(apogee_time, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % Add vertical line at apogee
xlabel('Time (s)');
ylabel('Height (m)');
legend('Estimated Height', 'Actual Height', 'Apogee Time', 'Location', 'southeast'); % Legend in bottom right
title('Rocket Altitude: Estimated vs Actual');
grid on;

subplot(3,1,2);
plot(0:Ts:(simulationTime-Ts), velocityLog, 'b', 'LineWidth', 1.5); hold on;
plot(0:Ts:(simulationTime-Ts), actualVelocityLog, 'r--', 'LineWidth', 1.5);
xline(apogee_time, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % Add vertical line at apogee
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Estimated Velocity', 'Actual Velocity', 'Apogee Time');
title('Rocket Velocity: Estimated vs Actual');
grid on;

subplot(3,1,3);
plot(0:Ts:(simulationTime-Ts), accelerationLog, 'b', 'LineWidth', 1.5); hold on;
plot(0:Ts:(simulationTime-Ts), actualAccelerationLog, 'r--', 'LineWidth', 1.5);
xline(apogee_time, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % Add vertical line at apogee
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Estimated Acceleration', 'Actual Acceleration', 'Apogee Time');
title('Rocket Acceleration: Estimated vs Actual');
grid on;


% Figure 2: Flap Angle, Thrust, and Mass over Time
figure;
subplot(3,1,1);
aperture_percentage = rad2deg(flapLog) / 180 * 100;
plot(0:Ts:(simulationTime-Ts), aperture_percentage, 'LineWidth', 1.5);
xline(apogee_time, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Aperture (%)');
title('Airbrake Aperture over Time');
ylim([0 100]);
grid on;


subplot(3,1,2);
plot(0:Ts:(simulationTime-Ts), thrustLog, 'LineWidth', 1.5);
xline(apogee_time, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % Add vertical line at apogee
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Thrust over Time');
grid on;

subplot(3,1,3);
plot(0:Ts:(simulationTime-Ts), massLog, 'LineWidth', 1.5);
xline(apogee_time, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % Add vertical line at apogee
xlabel('Time (s)');
ylabel('Mass (kg)');
title('Rocket Mass over Time');
grid on;
