function [A,B,C,D] = bicycle_model(params,Vx)
%BICYCLE_MODEL Linear single-track lateral vehicle model.
%
% States:
%   x = [v_y; r]
%
% Input:
%   delta = front steering angle
%
% Parameters:
%   m   vehicle mass
%   Iz  yaw inertia
%   lf  CG to front axle
%   lr  CG to rear axle
%   Cf  front cornering stiffness
%   Cr  rear cornering stiffness
%
% Vx must be positive.

m  = params.m;
Iz = params.Iz;
lf = params.lf;
lr = params.lr;
Cf = params.Cf;
Cr = params.Cr;

Vx = max(Vx,0.1);

A = [-(Cf+Cr)/(m*Vx),        -Vx-(Cf*lf-Cr*lr)/(m*Vx);
     -(Cf*lf-Cr*lr)/(Iz*Vx), -(Cf*lf^2+Cr*lr^2)/(Iz*Vx)];

B = [Cf/m;
     Cf*lf/Iz];

C = eye(2);
D = zeros(2,1);
end
