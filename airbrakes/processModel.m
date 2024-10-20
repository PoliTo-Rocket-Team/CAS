function x_next = processModel(x, delta, Ts, params, mass)
    % processModel predicts the next state of the rocket based on the current state.
    %
    % Parameters:
    %   x - The current state vector [altitude; velocity; acceleration].
    %   delta - The flap angle (radians) affecting drag calculations.
    %   Ts - The time step for the prediction (seconds).
    %   params - A struct containing parameters related to the rocket's environment and dynamics.
    %   mass - The current mass of the rocket (kg).
    %
    % Output:
    %   x_next - The predicted next state vector [altitude; velocity; acceleration].
    
    % Extract parameters for calculations
    g = params.g;  % Gravitational acceleration (m/s^2)
    rho = params.rho;  % Air density (kg/m^3)
    Cd_f = params.Cd_f;  % Drag coefficient of the flaps
    Cd_R = params.Cd_R;  % Drag coefficient of the rocket body
    A_f = params.A_f;  % Reference area of the flaps (m^2)
    A_R = params.A_R;  % Reference area of the rocket body (m^2)
    n_flaps = params.n_flaps;  % Number of control flaps

    v = x(2);  % Extract velocity from state vector
    
    % ** Linear mapping of delta to aperture **
    % delta ranges from 0 to pi (0 to 180 degrees), linearly mapped to 0 to 1
    aperture = delta / pi;

    % Calculate total drag force acting on the rocket due to flaps and body
    drag_force = 0.5 * rho * (n_flaps * Cd_f * A_f * aperture + Cd_R * A_R) * v^2;
    
    % Compute acceleration
    a = -drag_force / mass - g; % Consider drag and gravity
    
    % Predict next state
    h_next = x(1) + v * Ts + 0.5 * a * Ts^2; % Update altitude using kinematics
    v_next = x(2) + a * Ts; % Update velocity
    
    % Create the predicted state vector [altitude; velocity; acceleration]
    x_next = [h_next; v_next; a];
end
