function mass = rocketMass(t, burn_time, m_start, m_end)
    % rocketMass models the decrease in rocket mass during fuel burn.
    %
    % The mass decreases linearly from m_start (initial mass) to m_end (final mass) 
    % during the specified burn time. After the burn time, the mass remains constant at m_end.
    %
    % Parameters:
    %   t - The current time in seconds (should be non-negative).
    %   burn_time - The total burn time in seconds, representing the duration of fuel burn.
    %   m_start - The rocket's initial mass at launch (kg).
    %   m_end - The rocket's final mass after fuel depletion (kg).
    %
    % Output:
    %   mass - The calculated rocket's mass at time t (kg).

    % Validate input for burn time and current time
    if burn_time <= 0
        error('Burn time must be a positive value.');
    end
    if t < 0
        error('Current time must be non-negative.');
    end
    
    % Check if the current time is within the burn time period
    if t <= burn_time
        % During burn: Linearly interpolate the mass between m_start and m_end
        mass = m_start - (m_start - m_end) * (t / burn_time); 
    else
        % After burn: Mass remains constant at m_end
        mass = m_end; 
    end
end
