% Produced by CVXGEN, 2020-12-16 04:35:30 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
Fd = params.Fd;
T0 = params.T0;
dotx = params.dotx;
tau = params.tau;
varepsilon = params.varepsilon;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable Fc(12, 1);

  minimize(quad_form(Fc - Fd, eye(12)));
  subject to
    -tau*Fc'*dotx >=  - T0 + varepsilon;
cvx_end
vars.Fc = Fc;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
