function mu = burckhardt_longitudinal(lambda,params)
%BURCKHARDT_LONGITUDINAL Generic Burckhardt-style friction coefficient.
%
% mu(lambda) = c1*(1-exp(-c2*lambda)) - c3*lambda
%
% params.c1, params.c2, params.c3 define a chosen road condition.

c1 = params.c1;
c2 = params.c2;
c3 = params.c3;

mu = c1*(1-exp(-c2*lambda)) - c3*lambda;
end
