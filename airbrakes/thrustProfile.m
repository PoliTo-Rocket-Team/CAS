function [time, thrust] = thrustProfile(filename)
    % thrustProfile reads the thrust profile from a CSV file, ignoring any non-numeric headers.
    %
    % The CSV file is expected to contain two columns:
    %   - First column: Time (seconds)
    %   - Second column: Thrust (Newtons)
    %
    % Parameters:
    %   filename - The name of the CSV file containing the thrust profile.
    %
    % Outputs:
    %   time - A vector of time points (in seconds).
    %   thrust - A vector of corresponding thrust values (in Newtons).
    %
    % The function ensures that only valid numeric values are extracted and handles
    % any missing or invalid data by removing rows containing NaN or Inf values.

    % Detect import options to skip non-numeric headers and set data reading preferences
    opts = detectImportOptions(filename);
    opts.DataLines = 5; % Skip the first 4 lines (assumed to be metadata or headers)
    opts.VariableNamingRule = 'preserve'; % Preserve the original column names for clarity

    try
        % Attempt to read the thrust profile data into a table
        data = readtable(filename, opts);
    catch
        % If an error occurs (e.g., file missing or improperly formatted), display a meaningful message
        error('Error reading the thrust profile file. Ensure the file exists and is formatted correctly.');
    end

    % Extract the time (first column) and thrust (second column) from the table
    time = data{:, 1}; % Extract time data in seconds from the first column
    thrust = data{:, 2}; % Extract thrust data in Newtons from the second column

    % Ensure valid numeric values by removing rows with non-finite values (e.g., NaN, Inf)
    valid_idx = isfinite(time) & isfinite(thrust); % Create a logical index for valid entries
    time = time(valid_idx); % Filter time to include only valid entries
    thrust = thrust(valid_idx); % Filter thrust to include only valid entries

    % Optional: Check if the resulting data is empty and warn if necessary
    if isempty(time) || isempty(thrust)
        warning('Thrust profile data is empty or contains invalid entries after processing.');
    end
end
