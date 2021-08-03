% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(Fc - Fd, eye(12)))
%   subject to
%     -tau*Fc'*dotx >=  - T0 + varepsilon
%
% with variables
%       Fc  12 x 1
%
% and parameters
%       Fd  12 x 1
%       T0   1 x 1
%     dotx  12 x 1
%      tau   1 x 1
% varepsilon   1 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.Fd, ..., params.varepsilon, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2020-12-16 04:35:30 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
