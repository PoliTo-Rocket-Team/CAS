function z = measurementModel(x)
    % measurementModel maps the state vector to the measurement space.
    %
    % Parameters:
    %   x - The state vector [altitude; velocity; acceleration].
    %
    % Output:
    %   z - The measurement vector [altitude; acceleration].
    
    z = [x(1); x(3)]; % Measurement includes altitude and acceleration
end
