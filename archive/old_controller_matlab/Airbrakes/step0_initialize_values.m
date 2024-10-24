clc
clear all
close all

% MASTER WARNING
% the PID is tuned for the parameters (if not previously modified by the
% user) which can be loaded through the sim_start.m script, and as such any
% modification in the variables which will be generated through this 3-step
% process might cause a suboptimal response

% therefore, in case of substantial modifications to the simulation
% variables, further tuning of the PID is required

% additional info on the PID tuning process can be found on the README.md
% located in the Airbrakes folder

% INFO
% the scope of this file is generating the 1x1 timeserie variables which
% correlate the thrust of the engine and the mass of the whole rocket
% assembly over time

% the thrust_ts and mass_ts timetables are
% needed to run step1_simtime_calculator_script.m,
% step2_trajectories_sim_script.m and ultimately the Airbrake_sim.slx
% itself

% EDIT: this script now also generates the variable which indicates the
% rate limit of the actuator and the launch attitude

% alternatively, the rocketdata.mat file can be loaded to load the
% aforementioned variables without the need of running this code

rate_lim = 1.85; % rate limit of the actuator
launch_attitude = pi; % launch attitude of VES

thrust_table = readmatrix("Cesaroni_8088M1790-P.csv"); % modified
thrust = thrust_table(:,2);
thrust_time = thrust_table(:,1);

time = linspace(0,30,1000); % modified
thrust_interp = interp1(thrust_time,thrust,time); % interpolate where needed
thrust_interp(isnan(thrust_interp)) = 0;

thrust_ts = timeseries(thrust_interp',time);
mass_motor = 8.298; % initial mass of the motor assembly
motor_dry = 3.481; % final mass of the motor assembly
mass_diff = mass_motor-motor_dry;

t_burn = 4.625; % burn time of the engine
j = 1;

for mass_struct = 18.245 % mass of the rocket without the engine assembly % optimal 25.845
        m_init = mass_struct + mass_motor;
        m_final = m_init - mass_diff;
        k = -mass_diff/t_burn;
        i = 1;
        mass = ones(length(time),1)*m_final;
        while time(i) < t_burn
            mass(i) = k*time(i) + m_init; % linear decrease of total mass during burn time of motor
            i = i + 1;
        end
        mass_ts = timeseries(mass,time);
end


figure(1), hold on, grid on
plot(mass_ts.time,mass_ts.data,'r')
ylabel('Mass (kg)'), xlabel('Time (s)')

figure(2), hold on, grid on
plot(thrust_ts.time,thrust_ts.data,'r')
ylabel('Thrust (N)'), xlabel('Time (s)')

disp('thrust_ts saved to workspace');
fprintf('rate_lim saved to rocketdata.mat \n');
fprintf('launch_attitude saved to rocketdata.mat \n');
fprintf('mass_ts saved to workspace \n\n');

answer = input('do you want to save the variables to rocketdata.mat? (yes/[no]) \n->', 's');

if isempty(answer) || strcmpi(answer, 'no')
    fprintf('\n');
else
    save rocketdata thrust_ts mass_ts rate_lim launch_attitude
    fprintf('\n');
    fprintf('rate_lim saved to rocketdata.mat \n');
    fprintf('launch_attitude saved to rocketdata.mat \n');
    fprintf('thrust_ts saved to rocketdata.mat \n');
    fprintf('mass_ts saved to rocketdata.mat \n\n');
end

clearvars -except thrust_ts mass_ts rate_lim launch_attitude

disp('done, proceed to step 1');