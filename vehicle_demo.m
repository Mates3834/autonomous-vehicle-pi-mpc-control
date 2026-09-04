function vehicle_demo()
%VEHICLE_DEMO Small generic longitudinal/lateral control demonstration.

close all; clc;

%% Longitudinal PI example
dt = 0.01;
t = 0:dt:12;

v = 0;
vLog = zeros(size(t));
uLog = zeros(size(t));

state.integral = 0;
pPI.Kp = 0.8;
pPI.Ki = 0.5;
pPI.Kaw = 2.0;
pPI.uMin = -1;
pPI.uMax = 1;

for k = 1:numel(t)
    if t(k) < 6
        vRef = 15;
    else
        vRef = 10;
    end

    [u,state] = pi_speed_controller(vRef,v,dt,state,pPI);

    % Generic first-order longitudinal plant
    vDot = -0.25*v + 5*u;
    v = v + dt*vDot;

    vLog(k) = v;
    uLog(k) = u;
end

figure;
plot(t,vLog,'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('Speed');
title('Generic PI Speed-Control Example');

%% Lateral model + MPC
veh.m = 1500;
veh.Iz = 2500;
veh.lf = 1.2;
veh.lr = 1.6;
veh.Cf = 70000;
veh.Cr = 70000;

Vx = 15;
[A,B,C,~] = bicycle_model(veh,Vx);
Ts = 0.05;
sysd = c2d(ss(A,B,[0 1],0),Ts);
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

paramsMPC.N = 10;
paramsMPC.Qy = 10;
paramsMPC.Ru = 2;
paramsMPC.uMin = deg2rad(-30);
paramsMPC.uMax = deg2rad(30);

x = [0;0];
Nsim = 150;
r = 0.08*ones(paramsMPC.N,1);

rLog = zeros(Nsim,1);
yLog = zeros(Nsim,1);
uMpcLog = zeros(Nsim,1);

for k = 1:Nsim
    uMpc = mpc_controller(Ad,Bd,Cd,x,r,paramsMPC);
    x = Ad*x + Bd*uMpc;

    yLog(k) = Cd*x;
    rLog(k) = r(1);
    uMpcLog(k) = uMpc;
end

figure;
tt = (0:Nsim-1)*Ts;
plot(tt,yLog,'LineWidth',1.2); hold on;
plot(tt,rLog,'--','LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
legend('Response','Reference');
title('Generic Lateral MPC Example');
end
