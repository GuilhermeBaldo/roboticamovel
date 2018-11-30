function P3DX = Robot_Plot_3D(x,y,phi,color)

global Robot

% ROBOT_PLOT_3D plotes the "Pioneer" robot in the framework with the pose
%     established on input parameters. This function is used during 
%     simulation to show the robot behavior in the space work.
%     
%     P3DX = Robot_Plot_3D(x,y,phi,color)
%     
%     x,y,phi = robot pose
%     color = color used to represent the robot. Default: 'b' or [0 0 1].

if nargin < 4
    color = 'b';
end

%%% Matriz de Rotacao
Rot = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];

Cir = Rot*Robot.Ci;
Fir = Rot*Robot.Fi;

Csr = Rot*Robot.Cs;
Fsr = Rot*Robot.Fs;

Fa1r = Rot*Robot.Fa1;
Fa2r = Rot*Robot.Fa2;
Fa3r = Rot*Robot.Fa3;
Fa4r = Rot*Robot.Fa4;
Fa5r = Rot*Robot.Fa5;
Fa6r = Rot*Robot.Fa6;
Fa7r = Rot*Robot.Fa7;
Fa8r = Rot*Robot.Fa8;

Rodadr = Rot*Robot.Rodad;
Rodaer = Rot*Robot.Rodae;

P3DX(1)  = patch(Fir(1,:) +x, Fir(2,:) +y, Fir(3,:),  [0 0 1]);
P3DX(2)  = patch(Cir(1,:) +x, Cir(2,:) +y, Cir(3,:),  [1 1 1]);
P3DX(3)  = patch(Fsr(1,:) +x, Fsr(2,:) +y, Fsr(3,:),  color);
P3DX(4)  = patch(Csr(1,:) +x, Csr(2,:) +y, Csr(3,:),  [1 1 1]);
P3DX(5)  = patch(Fa1r(1,:)+x, Fa1r(2,:)+y, Fa1r(3,:), [0.7 0 0]);
P3DX(6)  = patch(Fa2r(1,:)+x, Fa2r(2,:)+y, Fa2r(3,:), [0.7 0 0]);
P3DX(7)  = patch(Fa3r(1,:)+x, Fa3r(2,:)+y, Fa3r(3,:), [0.7 0 0]);
P3DX(8)  = patch(Fa4r(1,:)+x, Fa4r(2,:)+y, Fa4r(3,:), [0.7 0 0]);
P3DX(9)  = patch(Fa5r(1,:)+x, Fa5r(2,:)+y, Fa5r(3,:), [0.7 0 0]);
P3DX(10) = patch(Fa6r(1,:)+x, Fa6r(2,:)+y, Fa6r(3,:), [0.7 0 0]);
P3DX(11) = patch(Fa7r(1,:)+x, Fa7r(2,:)+y, Fa7r(3,:), [0.7 0 0]);
P3DX(12) = patch(Fa8r(1,:)+x, Fa8r(2,:)+y, Fa8r(3,:), [0.7 0 0]);
P3DX(13) = patch(Rodadr(1,:)+x,Rodadr(2,:)+y,Rodadr(3,:),[0 0 0]);
P3DX(14) = patch(Rodaer(1,:)+x,Rodaer(2,:)+y,Rodaer(3,:),[0 0 0]);

  xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    