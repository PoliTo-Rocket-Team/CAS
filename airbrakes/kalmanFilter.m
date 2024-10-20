function [x_est, P] = kalmanFilter(x_est, P, z, u, F, B, H, Q, R)
    % kalmanFilter implements a Kalman filter for estimating state variables,
    % including altitude, velocity, and acceleration.
    %
    % Parameters:
    %   x_est - The current state estimate (vector) [height; velocity; acceleration].
    %   P - The current estimate covariance matrix (square matrix).
    %   z - The measurement vector (observations) [height measurement; velocity measurement].
    %   u - The control input vector (forces or other inputs affecting the system).
    %   F - The state transition matrix (describes how the state evolves over time).
    %   B - The control input matrix (describes how control inputs affect the state).
    %   H - The observation matrix (describes how the state relates to the measurements).
    %   Q - The process noise covariance matrix (uncertainty in the process model).
    %   R - The measurement noise covariance matrix (uncertainty in the measurements).
    %
    % Outputs:
    %   x_est - The updated state estimate after prediction and measurement update.
    %   P - The updated estimate covariance matrix.

    % Prediction step
    % Predict the next state based on the current state and control input
    x_pred = F * x_est + B * u;  % Predicted state [height, velocity, acceleration]
    
    % Predict the next covariance matrix based on the current covariance and process noise
    P_pred = F * P * F' + Q;  % Predicted covariance matrix

    % Update step
    % Calculate the Kalman gain to weigh the measurement versus the prediction
    K = P_pred * H' / (H * P_pred * H' + R);  % Kalman gain (matrix that balances prediction and measurement)

    % Update the state estimate using the measurement residual (z - H * x_pred)
    x_est = x_pred + K * (z - H * x_pred);  % Updated state estimate combining prediction and new measurement
    
    % Update the covariance matrix to reflect the uncertainty after measurement update
    P = (eye(size(K, 1)) - K * H) * P_pred;  % Updated covariance matrix
end
