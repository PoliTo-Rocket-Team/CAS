function optimal_delta = findOptimalFlapAngle(h, v, target_apogee, m, Cd_f, A_f, Cd_R, A_R, g, rho, Ts, n_flaps, current_delta, max_delta_rate, tol)
    % findOptimalFlapAngle determines the optimal flap angle needed to achieve a specified target apogee 
    % using a binary search approach, while respecting the flap angle rate limits.
    %
    % Parameters:
    %   h - Current height of the rocket (meters)
    %   v - Current velocity of the rocket (meters/second)
    %   target_apogee - Desired maximum height of the rocket (meters)
    %   m - Current mass of the rocket (kilograms)
    %   Cd_f - Drag coefficient for the control flaps (dimensionless)
    %   A_f - Reference area of the flaps (square meters)
    %   Cd_R - Drag coefficient for the rocket body (dimensionless)
    %   A_R - Reference area of the rocket body (square meters)
    %   g - Gravitational acceleration (meters/second^2)
    %   rho - Air density (kilograms/cubic meter)
    %   Ts - Simulation time step (seconds)
    %   n_flaps - Number of control flaps on the rocket (dimensionless)
    %   current_delta - Current servo angle (radians)
    %   max_delta_rate - Maximum allowed rate of change for servo angle (radians/second)
    %   tol - Tolerance for the binary search algo (degrees)
    %
    % Output:
    %   optimal_delta - Optimal flap angle (radians) to achieve the target apogee

    % Define the search range for the flap angle (0 to 90 degrees converted to radians)
    delta_min = 0;                 % Minimum servo angle (radians)
    delta_max = pi;                % Maximum servo angle (180 degrees in radians)
    tolerance = deg2rad(tol);      % Convert the allowable tolerance from degrees to radians
    
    % Perform binary search to find the optimal flap angle
    while (delta_max - delta_min) > tolerance
        % Calculate the midpoint flap angle for testing
        test_delta = (delta_min + delta_max) / 2;
        
        % Predict the apogee using the current test flap angle
        predicted_apogee = predictApogee(h, v, current_delta, test_delta, max_delta_rate, m, Cd_f, A_f, Cd_R, A_R, g, rho, Ts, n_flaps);
        
        % Adjust the search range based on the predicted apogee
        if predicted_apogee > target_apogee
            % If the predicted apogee is greater than the target, increase the minimum flap angle
            delta_min = test_delta; 
        else
            % If the predicted apogee is less than the target, decrease the maximum flap angle
            delta_max = test_delta; 
        end
    end
    
    % The optimal flap angle is the midpoint of the final search range
    optimal_delta = (delta_min + delta_max) / 2; 
end
