%%% Olivier Leveque & Maxime Maurin --  7 June 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xt, yt, ut, t] = trackingMPC(ssM, param_constraints, param_objective, param_simulation)

%parameters of the Building Model
A = ssM.A;
Bu = ssM.Bu;
Bd = ssM.Bd;
C = ssM.C;

%other parameters
nx = length(A); %number of states
ny = size(C,1); %number of outputs
nu = size(Bu,2); %number of inputs
nd = size(Bd,2); %number of disturbances

%define simulation parameters
N = param_simulation.N;
T = param_simulation.T;

%define objective parameters
R = param_objective.R;
yref = param_objective.yref;

%define constraints matrix
Mu = param_constraints.Mu;
My = param_constraints.My;
mu = param_constraints.mu;
my = param_constraints.my;

%define optimization variables
x = sdpvar(nx, N, 'full'); %states
x0 = sdpvar(nx, 1, 'full'); %initial states
y = sdpvar(ny, N, 'full'); %outputs
u = sdpvar(nu, N, 'full'); %inputs
d = sdpvar(nd, N, 'full'); %disturbances
    
%define constraints and objective
constraints = [];
objective = 0;
for i = 1:N-1
    if i == 1 
        constraints = [constraints, x(:,i+1) == A*x0 + Bu*u(:,i) + Bd*d(:,i)]; %system dynamics - first step
    else
        constraints = [constraints, x(:,i+1) == A*x(:,i) + Bu*u(:,i) + Bd*d(:,i)]; %system dynamics
    end
    constraints = [constraints, y(:,i) == C*x(:,i)]; %observability
    constraints = [constraints, Mu*u(:,i) <= mu]; %input constraints
    constraints = [constraints, My*y(:,i) <= my]; %output constraints
    objective = objective + (y(:,i)-yref)'*R*(y(:,i)-yref); %cost function
end
constraints = [constraints, y(:,N) == C*x(:,N)]; %observability
constraints = [constraints, Mu*u(:,N) <= mu]; %terminal input constraints
constraints = [constraints, My*y(:,N) <= my]; %terminal output constraints
objective = objective + (y(:,N)-yref)'*R*(y(:,N)-yref); %terminal cost

%create a controller
ops = sdpsettings('verbose', 1);
parameters_in = [x0; d(:)];
solutions_out = u;
controller = optimizer(constraints, objective, ops, parameters_in, solutions_out);

%simulate the system
option = 1; %simulation with no night-setbacks and no variable cost
[xt, yt, ut, t, ~] = simBuild(controller, T, @shiftPred, N, option);

end