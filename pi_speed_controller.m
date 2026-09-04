function [u, state] = pi_speed_controller(vRef, v, dt, state, params)
%PI_SPEED_CONTROLLER Generic PI speed controller with optional anti-windup.
%
% Inputs:
%   vRef   desired speed
%   v      measured speed
%   dt     sample time
%   state.integral
%
% params:
%   Kp, Ki, uMin, uMax, Kaw
%
% Output:
%   u      saturated actuator command

if ~isfield(state,'integral')
    state.integral = 0;
end

e = vRef - v;

uUnsat = params.Kp*e + params.Ki*state.integral;
u = min(max(uUnsat,params.uMin),params.uMax);

% Back-calculation anti-windup
state.integral = state.integral + dt*(e + params.Kaw*(u-uUnsat));
end
