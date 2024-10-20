function F = calculateJacobianF(x_est, delta, params, Ts, mass)
    % calculateJacobianF computes the Jacobian matrix for the extended Kalman filter.
    %
    % Parameters:
    %   x_est - The current state estimate (vector) [height; velocity; acceleration].
    %   delta - The flap angle (radians), influencing drag calculations.
    %   params - A struct containing parameters related to the rocket's environment and dynamics.
    %   Ts - The time step for the prediction (seconds).
    %   mass - The current mass of the rocket (kg).
    %
    % Output:
    %   F - The Jacobian matrix (state transition matrix) for the Kalman filter.

    % Extract relevant parameters for calculations
    v = x_est(2);  % Current velocity from state vector
    rho = params.rho;  % Air density (kg/m^3)
    Cd_f = params.Cd_f;  % Drag coefficient of the flaps
    Cd_R = params.Cd_R;  % Drag coefficient of the rocket body
    A_f = params.A_f;  % Reference area of the flaps (m^2)
    A_R = params.A_R;  % Reference area of the rocket body (m^2)
    n_flaps = params.n_flaps;  % Number of control flaps

    % Partial derivative of drag force with respect to velocity
    d_drag_dv = rho * v * (n_flaps * Cd_f * A_f * sin(delta) + Cd_R * A_R);
    
    % Jacobian matrix (partial derivatives of [height, velocity, acceleration] w.r.t [h, v, a])
    F = [1 Ts 0.5 * Ts^2;  % Relationship for height update
         0 1 Ts;          % Relationship for velocity update
         0 -d_drag_dv / mass 0];  % Relationship for acceleration update, using current mass
end
