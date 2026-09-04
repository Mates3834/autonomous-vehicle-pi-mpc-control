function [deltaRef,kappa,targetPoint] = pure_pursuit_reference(pos,psi,path,lookahead,wheelbase)
%PURE_PURSUIT_REFERENCE Generic Pure Pursuit steering reference.
%
% pos       [x y]
% psi       vehicle heading [rad]
% path      N-by-2 [x y]
% lookahead look-ahead distance
% wheelbase wheelbase length

d = vecnorm(path - pos,2,2);
idx = find(d >= lookahead,1,'first');

if isempty(idx)
    idx = size(path,1);
end

targetPoint = path(idx,:);
alpha = wrapToPi(atan2(targetPoint(2)-pos(2),targetPoint(1)-pos(1)) - psi);

kappa = 2*sin(alpha)/lookahead;
deltaRef = atan(wheelbase*kappa);
end
