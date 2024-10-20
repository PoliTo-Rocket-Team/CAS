function sigmaPoints = generateSigmaPoints(x_est, P, lambda)
    % generateSigmaPoints computes the sigma points for the Unscented Kalman Filter.
    %
    % Parameters:
    %   x_est - The current state estimate (vector).
    %   P - The current estimate covariance matrix (square matrix).
    %   lambda - The scaling parameter for sigma points.
    %
    % Output:
    %   sigmaPoints - The generated sigma points (matrix).
    
    L = numel(x_est); % State dimension
    sigmaPoints = zeros(L, 2 * L + 1);
    sigmaPoints(:, 1) = x_est; % First sigma point is the mean
    
    sqrtP = chol((L + lambda) * P, 'lower'); % Square root of scaled covariance
    
    for i = 1:L
        sigmaPoints(:, i + 1) = x_est + sqrtP(:, i);   % Positive sigma points
        sigmaPoints(:, i + L + 1) = x_est - sqrtP(:, i); % Negative sigma points
    end
end
