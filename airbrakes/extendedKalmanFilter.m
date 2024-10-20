function [x_est, P] = extendedKalmanFilter(x_est, P, z, delta, Ts, Q, R, params, mass)
    % extendedKalmanFilter implements an Extended Kalman Filter for estimating 
    % the state of a rocket including altitude, velocity, and acceleration.
    %
    % Parameters:
    %   x_est - The current state estimate (vector) [height; velocity; acceleration].
    %   P - The current estimate covariance matrix (square matrix).
    %   z - The measurement vector (observations) [height measurement; acceleration measurement].
    %   delta - The flap angle (radians), which affects the drag force.
    %   Ts - The time step for the prediction (seconds).
    %   Q - The process noise covariance matrix (uncertainty in the process model).
    %   R - The measurement noise covariance matrix (uncertainty in the measurements).
    %   params - A struct containing parameters related to the rocket's environment and dynamics.
    %   mass - The current mass of the rocket (kg).
    %
    % Outputs:
    %   x_est - The updated state estimate after prediction and measurement update.
    %   P - The updated estimate covariance matrix.

    % Extract parameters for readability
    g = params.g;  % Gravitational acceleration (m/s^2)
    rho = params.rho;  % Air density (kg/m^3)
    Cd_f = params.Cd_f;  % Drag coefficient of the flaps
    Cd_R = params.Cd_R;  % Drag coefficient of the rocket body
    A_f = params.A_f;  % Reference area of the flaps (m^2)
    A_R = params.A_R;  % Reference area of the rocket body (m^2)
    n_flaps = params.n_flaps;  % Number of control flaps

    % Nonlinear process model
    v = x_est(2);  % Extract the current velocity from the state vector

    % ** Linear mapping of delta to aperture **
    % delta ranges from 0 to pi (0 to 180 degrees), linearly mapped to 0 to 1
    aperture = delta / pi;

    % Calculate total drag force acting on the rocket due to flaps and body
    drag_force = 0.5 * rho * (n_flaps * Cd_f * A_f * aperture + Cd_R * A_R) * v^2;

    % Prediction step using current mass to account for dynamics
    a_pred = -drag_force / mass - g;  % Acceleration considering drag and gravity
    v_pred = x_est(2) + a_pred * Ts;  % Predicted velocity
    h_pred = x_est(1) + x_est(2) * Ts + 0.5 * a_pred * Ts^2;  % Predicted height using kinematics

    % Create the predicted state vector
    x_pred = [h_pred; v_pred; a_pred];

    % Calculate Jacobians (F and H matrices)
    F = calculateJacobianF(x_est, delta, params, Ts, mass);  % State transition Jacobian
    H = [1 0 0; 0 0 1];  % Measurement matrix (observing altitude and acceleration)

    % Kalman filter update (prediction + measurement update)
    P_pred = F * P * F' + Q;  % Predicted covariance matrix
    K = P_pred * H' / (H * P_pred * H' + R);  % Kalman gain
    x_est = x_pred + K * (z - H * x_pred);    % Update estimate with measurement
    P = (eye(size(K,1)) - K * H) * P_pred;    % Update covariance to reflect uncertainty
end
