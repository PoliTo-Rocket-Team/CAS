function [x_est, P] = unscentedKalmanFilter(x_est, P, z, delta, Ts, Q, R, params, mass)
    % unscentedKalmanFilter implements the Unscented Kalman Filter for estimating 
    % the state of a rocket, including altitude, velocity, and acceleration.
    %
    % Parameters:
    %   x_est - The current state estimate (vector) [height; velocity; acceleration].
    %   P - The current estimate covariance matrix (square matrix).
    %   z - The measurement vector (observations) [height measurement; acceleration measurement].
    %   delta - The flap angle (radians) affecting drag calculations.
    %   Ts - The time step for the prediction (seconds).
    %   Q - The process noise covariance matrix (uncertainty in the process model).
    %   R - The measurement noise covariance matrix (uncertainty in the measurements).
    %   params - A struct containing parameters related to the rocket's environment and dynamics.
    %   mass - The current mass of the rocket (kg).
    %
    % Outputs:
    %   x_est - The updated state estimate after prediction and measurement update.
    %   P - The updated estimate covariance matrix.

    % UKF parameters
    L = numel(x_est); % Number of states
    alpha = 1e-3; % Sigma point spread parameter
    beta = 2; % Parameter for Gaussian distributions
    kappa = 0; % Secondary scaling parameter
    lambda = alpha^2 * (L + kappa) - L; % Scaling factor
    
    % Weights for mean and covariance
    Wm = [lambda / (L + lambda), repmat(1 / (2 * (L + lambda)), 1, 2 * L)]; % Weights for mean
    Wc = Wm; 
    Wc(1) = Wc(1) + (1 - alpha^2 + beta); % Adjust first weight for covariance

    % 1. Generate sigma points
    sigmaPoints = generateSigmaPoints(x_est, P, lambda);
    
    % 2. Predict step: propagate sigma points through the process model
    sigmaPoints_pred = zeros(L, 2 * L + 1);
    for i = 1:(2 * L + 1)
        sigmaPoints_pred(:, i) = processModel(sigmaPoints(:, i), delta, Ts, params, mass);
    end
    
    % Predicted state mean
    x_pred = sum(Wm .* sigmaPoints_pred, 2);
    
    % Predicted covariance
    P_pred = Q; % Start with process noise
    for i = 1:(2 * L + 1)
        diff = sigmaPoints_pred(:, i) - x_pred; % Difference from predicted mean
        P_pred = P_pred + Wc(i) * (diff * diff'); % Update covariance
    end
    
    % 3. Update step: measurement update
    % Generate predicted measurement sigma points
    z_sigma = zeros(2, 2 * L + 1); % Measurement dimensions: altitude and acceleration
    for i = 1:(2 * L + 1)
        z_sigma(:, i) = measurementModel(sigmaPoints_pred(:, i));
    end
    
    % Predicted measurement mean
    z_pred = sum(Wm .* z_sigma, 2);
    
    % Innovation covariance
    P_zz = R; % Start with measurement noise
    for i = 1:(2 * L + 1)
        diff_z = z_sigma(:, i) - z_pred; % Difference from predicted measurement mean
        P_zz = P_zz + Wc(i) * (diff_z * diff_z'); % Update measurement covariance
    end
    
    % Cross covariance between state and measurement
    P_xz = zeros(L, 2); % 2 because we observe altitude and acceleration
    for i = 1:(2 * L + 1)
        diff_x = sigmaPoints_pred(:, i) - x_pred; % State difference
        diff_z = z_sigma(:, i) - z_pred; % Measurement difference
        P_xz = P_xz + Wc(i) * (diff_x * diff_z'); % Update cross covariance
    end
    
    % Kalman gain
    K = P_xz / P_zz;
    
    % Update state estimate and covariance
    x_est = x_pred + K * (z - z_pred); % Update state estimate
    P = P_pred - K * P_zz * K'; % Update covariance
end
