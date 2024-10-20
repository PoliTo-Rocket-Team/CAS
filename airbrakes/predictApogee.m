function h_max = predictApogee(h, v, current_delta, desired_delta, max_delta_rate, m, Cd_f, A_f, Cd_R, A_R, g, rho, Ts, n_flaps)
    % predictApogee predicts the rocket's maximum height (apogee) 
    % during unpowered ascent, considering rate-limited adjustments 
    % of flap angles for drag control.
    %
    % Parameters:
    %   h - Current height of the rocket (meters)
    %   v - Current velocity of the rocket (meters/second)
    %   current_delta - Current servo angle (radians)
    %   desired_delta - Desired servo angle to set (radians)
    %   max_delta_rate - Maximum allowed rate of change for flap angle (radians/second)
    %   m - Current mass of the rocket (kilograms)
    %   Cd_f - Drag coefficient for the control flaps (dimensionless)
    %   A_f - Reference area of the flaps (square meters)
    %   Cd_R - Drag coefficient for the rocket body (dimensionless)
    %   A_R - Reference area of the rocket body (square meters)
    %   g - Gravitational acceleration (meters/second^2)
    %   rho - Air density (kilograms/cubic meter)
    %   Ts - Simulation time step (seconds)
    %   n_flaps - Number of control flaps on the rocket (dimensionless)
    %
    % Output:
    %   h_max - Predicted apogee (maximum height) of the rocket (meters)

    % Initialize the current state variables
    h_current = h;         % Initialize current height with the initial height
    v_current = v;         % Initialize current velocity with the initial velocity
    delta = current_delta; % Initialize current flap angle

    % Integrate the rocket's motion until it begins to descend (velocity becomes negative)
    while v_current > 0
        % Calculate the required change in flap angle to reach the desired delta
        delta_change = desired_delta - delta; % Difference between desired and current flap angle
        max_change = max_delta_rate * Ts;     % Maximum allowable change in flap angle for this time step

        % Update the flap angle based on the maximum allowed change
        if abs(delta_change) > max_change
            % Limit the flap angle change to the maximum rate
            delta = delta + sign(delta_change) * max_change; 
        else
            % If within the allowed rate, set to the desired delta
            delta = desired_delta; 
        end

        % ** Linear mapping of delta to aperture **
        % delta ranges from 0 to pi (0 to 180 degrees), linearly mapped to 0 to 1
        aperture = delta / pi;

        % Calculate total drag force acting on the rocket due to flaps and body
        drag_force = 0.5 * rho * (n_flaps * Cd_f * A_f * aperture + Cd_R * A_R) * v_current^2;

        % Calculate the net acceleration due to drag and gravity
        acceleration = -drag_force / m - g; % Total acceleration acting on the rocket

        % Update velocity using forward Euler integration method
        v_current = v_current + acceleration * Ts;

        % Update height using forward Euler integration method
        h_current = h_current + v_current * Ts; 
    end

    % Return the predicted maximum height (apogee) when the rocket's velocity becomes zero
    h_max = h_current; 
end
