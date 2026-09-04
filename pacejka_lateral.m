function Fy = pacejka_lateral(alpha,Fz,params)
%PACEJKA_LATERAL Simplified Magic Formula lateral tire force.
%
% Fy = Fz * D * sin(C*atan(B*alpha - E*(B*alpha-atan(B*alpha))))
%
% alpha [rad]
% Fz    normal load

B = params.B;
C = params.C;
D = params.D;
E = params.E;

phi = B*alpha;
Fy = Fz .* D .* sin(C*atan(phi - E*(phi-atan(phi))));
end
