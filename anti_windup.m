function iDot = anti_windup(error,uSat,uUnsat,Kaw)
%ANTI_WINDUP Back-calculation anti-windup integrator correction.
iDot = error + Kaw*(uSat-uUnsat);
end
