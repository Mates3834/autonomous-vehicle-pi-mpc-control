function u = mpc_controller(Ad,Bd,Cd,x0,rSeq,params)
%MPC_CONTROLLER Generic constrained linear MPC using quadprog.
%
% Model:
%   x(k+1) = Ad*x(k) + Bd*u(k)
%   y(k)   = Cd*x(k)
%
% rSeq must contain N scalar references.
%
% params:
%   N, Qy, Ru, uMin, uMax
%
% Requires Optimization Toolbox.

N = params.N;
nx = size(Ad,1);
nu = size(Bd,2);

Phi = zeros(N,nx);
Gamma = zeros(N,N*nu);

for i = 1:N
    Phi(i,:) = Cd*(Ad^i);
    for j = 1:i
        Gamma(i,(j-1)*nu+1:j*nu) = Cd*(Ad^(i-j))*Bd;
    end
end

Qbar = params.Qy*eye(N);
Rbar = params.Ru*eye(N*nu);

H = 2*(Gamma'*Qbar*Gamma + Rbar);
f = 2*Gamma'*Qbar*(Phi*x0 - rSeq(:));

lb = params.uMin*ones(N*nu,1);
ub = params.uMax*ones(N*nu,1);

opts = optimoptions('quadprog','Display','off');
U = quadprog(H,f,[],[],[],[],lb,ub,[],opts);

if isempty(U)
    u = 0;
else
    u = U(1:nu);
end
end
