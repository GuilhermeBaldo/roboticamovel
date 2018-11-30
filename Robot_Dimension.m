function Robot_Dimension(scale)

% ROBOT_DIMENSION(scale) loads robot physical parameters.
%   scale = scale to be used to plot the robot. Default values are in
%   meters.

if nargin < 1
    scale = 1;
end    

et = 1.4;       % error factor for parameter estimates

Robot.theta = [0.26038*et 0.25095*et -0.00049969*et 0.99646*et 0.002629*et 1.0768*et];
Robot.theta_est = [0.26038 0.25095 -0.00049969 0.99646 0.002629 1.0768];
%Robot.theta_est = Robot.theta; % Exatct parameters

%%% Distance between the virtual axle and point of interessed
Robot.a = 0.2;

% Aceleration and speed limits for the Pioneer robot
Robot.up_max = 0.3; % [m/s^2]
Robot.wp_max = 1.745; % [rad/s^2]
Robot.u_max = 0.75; % [m/s]
Robot.w_max = 1.745; % [rad/s]

% Sample period of the robot servos and sensors.
Robot.Ta = 0.1;     % [s]

%%% Robot physical dimensions

Robot.Ci = [0.100 0.100 -0.100 -0.200 -0.200 -0.100; -0.150 0.150 0.150 0.075 -0.075 -0.150; 0.050 0.050 0.050 0.050 0.050 0.050]*scale;
Robot.Fi = [0.100 0.200 0.200 0.100; -0.150 -0.075 0.075 0.150; 0.050 0.050 0.050 0.050]*scale;

Robot.Cs = [0.100 0.100 -0.100 -0.200 -0.200 -0.100; -0.150 0.150 0.150 0.075 -0.075 -0.150; 0.210 0.210 0.210 0.210 0.210 0.210]*scale;
Robot.Fs = [0.100 0.200 0.200 0.100; -0.150 -0.075 0.075 0.150; 0.210 0.210 0.210 0.210]*scale;

%%% Faces do Robo 
Robot.Fa1 = [0.200 0.200 0.200 0.200; -0.075 0.075 0.075 -0.075; 0.210 0.210 0.050 0.050]*scale;
Robot.Fa2 = [0.200 0.100 0.100 0.200; 0.075 0.150 0.150 0.075; 0.210 0.210 0.050 0.050]*scale;
Robot.Fa3 = [0.100 -0.100 -0.100 0.100; 0.150 0.150 0.150 0.150; 0.210 0.210 0.050 0.050]*scale;
Robot.Fa4 = [-0.100 -0.200 -0.200 -0.100; 0.150 0.075 0.075 0.150; 0.210 0.210 0.050 0.050]*scale;
Robot.Fa5 = [-0.200 -0.200 -0.200 -0.200; 0.075 -0.075 -0.075 0.075; 0.210 0.210 0.050 0.050]*scale;
Robot.Fa6 = [-0.200 -0.100 -0.100 -0.200; -0.075 -0.150 -0.150 -0.075; 0.210 0.210 0.050 0.050]*scale;
Robot.Fa7 = [-0.100 0.100 0.100 -0.100; -0.150 -0.150 -0.150 -0.150; 0.210 0.210 0.050 0.050]*scale;
Robot.Fa8 = [0.100 0.200 0.200 0.100; -0.150 -0.075 -0.075 -0.150; 0.210 0.210 0.050 0.050]*scale;

%%% Roda do Robo
Robot.N = 12;
Robot.xyroda = 0.090*cos(0:2*pi/Robot.N:2*pi); 
Robot.zroda  = 0.090*(1 + sin(0:2*pi/Robot.N:2*pi));
Robot.Rodad   = [Robot.xyroda; -0.155*ones(size(Robot.xyroda)); Robot.zroda]*scale;
Robot.Rodae   = [Robot.xyroda; +0.155*ones(size(Robot.xyroda)); Robot.zroda]*scale;

global Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%