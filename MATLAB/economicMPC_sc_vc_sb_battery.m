%%% Olivier Leveque & Maxime Maurin --  7 June 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xt, yt, ut, t, et, xbt, total_cost] = economicMPC_sc_vc_sb_battery(ssM, ssModel, param_constraints, param_objective, param_simulation)

%parameters of the Building Model
A = ssM.A;
Bu = ssM.Bu;
Bd = ssM.Bd;
C = ssM.C;

%parameters of the Storage Model
a = ssModel.A;
b = ssModel.Bu;

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
S = param_objective.S;

%define constraints matrix
Mu = param_constraints.Mu;
My = param_constraints.My;
mu = param_constraints.mu;
my = param_constraints.my;

Mv = param_constraints.Mv;
Mxb = param_constraints.Mxb;
mv = param_constraints.mv;
mxb = param_constraints.mxb;

%define optimization variables
x = sdpvar(nx, N, 'full'); %states (building)
x0 = sdpvar(nx, 1, 'full'); %initial states (building)
xb = sdpvar(1, N, 'full'); %states (battery)
xb0 = sdpvar(1, 1, 'full'); %initial states (battery)
e = sdpvar(1, N, 'full'); %power consumption from the grid
v = sdpvar(1, N, 'full'); %charging power of the battery
u = sdpvar(nu, N, 'full'); %power consumption of the HVAC system
y = sdpvar(ny, N, 'full'); %outputs
d = sdpvar(nd, N, 'full'); %disturbances
epsilon = sdpvar(ny, N, 'full'); %slack variables
cp = sdpvar(1, N, 'full'); %variable prices
sb = sdpvar(2*ny, N, 'full'); %variable prices

%define constraints and objective
constraints = [];
objective = 0;
for i = 1:N-1
    if i == 1 
        constraints = [constraints, x(:,i+1) == A*x0 + Bu*u(:,i) + Bd*d(:,i)]; %system dynamics - first step
        constraints = [constraints, xb(i+1) == a*xb0 + b*v(i)]; %battery dynamics - first step
    else
        constraints = [constraints, x(:,i+1) == A*x(:,i) + Bu*u(:,i) + Bd*d(:,i)]; %system dynamics
        constraints = [constraints, xb(i+1) == a*xb(i) + b*v(i)]; %battery dynamics
    end
    constraints = [constraints, y(:,i) == C*x(:,i)]; %observability
    constraints = [constraints, Mu*u(:,i) <= mu]; %input constraints
    constraints = [constraints, My*y(:,i) <= my + sb(:,i) + kron([1; -1], epsilon(:,i))]; %soft output constraints
    constraints = [constraints, zeros(ny,1) <= epsilon(:,i)]; %slack variable constraints
    
    constraints = [constraints, v(i) == e(i) - sum(u(:,i))]; %charging power of the battery equation
    constraints = [constraints, Mv*v(:,i) <= mv]; %charging power of the battery contraints
    constraints = [constraints, Mxb*xb(:,i) <= mxb]; %battery state constraints
    constraints = [constraints, 0 <= e(i)]; %power consumption from the grid constraints
    
    objective = objective + cp(i)*e(i) + epsilon(:,i)'*S*epsilon(:,i); %economic cost function
end 
constraints = [constraints, y(:,N) == C*x(:,N)]; %observability
constraints = [constraints, Mu*u(:,N) <= mu]; %terminal input constraints
constraints = [constraints, My*y(:,N) <= my + sb(:,N) + kron([1; -1], epsilon(:,N))]; %terminal soft output constraints
constraints = [constraints, zeros(ny,1) <= epsilon(:,N)]; %terminal slack variable constraints

constraints = [constraints, v(N) == e(N) - sum(u(:,N))]; %charging power of the battery equation
constraints = [constraints, Mv*v(:,N) <= mv]; %terminal charging power of the battery contraints
constraints = [constraints, Mxb*xb(:,N) <= mxb]; %terminal battery state constraints
constraints = [constraints, 0 <= e(N)]; %terminal power consumption from the grid constraints

objective = objective + cp(N)*e(N) + epsilon(:,N)'*S*epsilon(:,N); %terminal economic cost function

%create a controller
ops = sdpsettings('verbose', 1, 'solver', '+gurobi');
parameters_in = [x0; xb0; d(:); cp(:); sb(:)];
solutions_out = [u; v; e];
controller = optimizer(constraints, objective, ops, parameters_in, solutions_out);

%simulate the system
[xt, yt, ut, t, et, xbt, total_cost] = simBuildStorage(controller, T, @shiftPred, N);

end