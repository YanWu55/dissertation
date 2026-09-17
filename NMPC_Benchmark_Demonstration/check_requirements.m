function check_requirements()
%CHECK_REQUIREMENTS Verify MATLAB toolboxes required by this demonstration.

v = ver;
names = string({v.Name});

hasMPC = any(names == "Model Predictive Control Toolbox");
hasOPT = any(names == "Optimization Toolbox");

if ~hasMPC
    error(['Model Predictive Control Toolbox is required. ' ...
        'The nonlinear MPC functionality is not available in this MATLAB installation.']);
end

if ~hasOPT
    error(['Optimization Toolbox is required by the default nonlinear MPC ' ...
        'nonlinear-programming solver.']);
end

if exist('nlmpc','file') == 0
    error('nlmpc was not found in this MATLAB installation.');
end

% nlmpcmove is an object function of nlmpc.
% Do not test it using exist('nlmpcmove','file').

fprintf('Toolbox check passed.\n');

end
