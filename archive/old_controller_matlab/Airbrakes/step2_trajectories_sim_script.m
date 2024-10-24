% clear all
% close all
clc

% WARNING
% this script needs mass_ts, thrust_ts, surfaceBlockParameters1, t2a,
% apogee and abcs_deploy to work

% generate them through the previous steps or load rocketdata.mat,
% MachExtension2Cdtable.mat and apogee_var.mat

% INFO
% the scope of this file is simulating the trajectory of the rocket at
% various air brakes extension values to then process the altitude and
% velocity of the rocket during flight for use in the airbrake_sim.slx
% simulation using the variables generated in the previous steps

% alternatively, the trajectories.mat file can be loaded to load the required
% variables without the need of running this code


t_int = 0.1; % trajectories step size

t_sim = t2a; % sim time (time-to-apogee)


mass_final = mass_ts.Data(end); % drymass of the rocket (to modify weight of the rocket edit mass_struct in step0_initialize_values.m)

time = linspace(0,t_sim,1000);
mass = ones(length(time),1)*mass_final;
mass_final_ts = timeseries(mass,time);

init_pos = [0 -apogee]; % initial position (target apogee)


dy = cell(1, 11);
dy_mat_raw = cell(1, 11);

y = cell(1, 11);
y_mat_raw = cell(1, 11);

t_raw = cell(1, 1);

for j=10:-1:0
    abe = j/10;
    

    i = 10 - j;

    out = sim('trajectories_sim.slx');


    out.vert_velocity.Data = flip(out.vert_velocity.Data);

    dy{i+1} = out.vert_velocity;
    dy_mat_raw{i+1} = out.vert_velocity.Data;
    

    figure(2), hold on
    plot(out.vert_velocity.Time, out.vert_velocity.Data, 'LineWidth', 0.2)


    out.vert_altitude.Data = flip(out.vert_altitude.Data);

    y{i+1} = out.vert_altitude;
    y_mat_raw{i+1} = out.vert_altitude.Data;

    figure(1), hold on
    plot(out.vert_altitude.Time, out.vert_altitude.Data, 'LineWidth', 0.2)
end

figure(2), hold on, grid on
xlabel('Time (s)')
ylabel('Velocity (m/s)')
ylim([-200 400])


figure(1), hold on, grid on
xlabel('Time (s)')
ylabel('Altitude (m)')
ylim([0 apogee+apogee/10])

y_mat = cell2mat(y_mat_raw);
dy_mat = cell2mat(dy_mat_raw);


t_raw{1} = out.vert_velocity.Time;
t = cell2mat(t_raw);

fprintf('trajectories variables saved to workspace \n\n');

answer = input('do you want to save the variables to trajectories.mat? (yes/[no]) \n->', 's');

if isempty(answer) || strcmpi(answer, 'no')
    fprintf('\n');
else
    save trajectories.mat t y y_mat dy dy_mat
    fprintf('\n');
    fprintf('trajectories variables saved to trajectories.mat \n\n');
end

clearvars -except rate_lim launch_attitude thrust_ts mass_ts t y y_mat dy dy_mat surfaceBlockParameters1 t2a apogee abcs_deploy

fprintf('done \n\n');
load pid_var.mat
fprintf('PID variables loaded \n\n');

disp('-> ready for simulation, running airbrake_sim.slx... <-');

% open Airbrake_sim.slx
out = sim('Airbrake_sim.slx');

figure(1), hold on
plot(out.sim_vertalt.time, out.sim_vertalt.data, 'r--', 'LineWidth', 1)
ylim([0 apogee+apogee/10])

figure(2), hold on
plot(out.sim_vertvel.time, out.sim_vertvel.data, 'r--', 'LineWidth', 1)
ylim([-200 400])

figure(3), hold on, grid on
plot(out.sim_abe.time, out.sim_abe.data, 'LineWidth', 1)
ylabel('Airbrakes Extension'), xlabel('Time (s)')