clear all
close all
clc

% INFO
% this script automatically loads all the variables required for
% simulation without the need of generating them through the single
% scripts, opens the Simulink schematic of the air brakes control system
% and runs it once

load pid_var.mat
load rocketdata.mat
load trajectories.mat
load apogee_var.mat
load MachExtension2Cdtable.mat

fprintf('required workspace variables loaded \n\n');
disp('-> ready for simulation, running airbrake_sim.slx... <-');

% open Airbrake_sim.slx
out = sim('Airbrake_sim.slx');

figure(1), hold on, grid on
plot(out.sim_vertalt.time, out.sim_vertalt.data, 'r')
xlabel('Time (s)')
ylabel('Altitude (m)')
yline(apogee,'--','Target Apogee','LabelHorizontalAlignment','left')

figure(2), hold on, grid on
plot(out.sim_vertvel.time, out.sim_vertvel.data, 'r')
xlabel('Time (s)')
ylabel('Velocity (m/s)')


figure(3), hold on, grid on
plot(out.sim_abe.time, out.sim_abe.data, 'r')
ylabel('Airbrakes Extension'), xlabel('Time (s)')