%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%              MPC -- Mini Project             %%%
%%%        Olivier Leveque & Maxime Maurin       %%%
%%%                  7 June 2016                 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
close all;

yalmip('clear');
clear all;

%% Model data

load building.mat;
load battery.mat;

% Parameters of the Building Model
A = ssM.A;
Bu = ssM.Bu;
Bd = ssM.Bd;
C = ssM.C;
Ts = ssM.timestep;

% Parameters of the Storage Model
a = ssModel.A;
b = ssModel.Bu;   

% Installation Test
yalmip('version')
sprintf('The Project files are successfully installed')

% Other parameters
nx = length(A); %number of states
ny = size(C,1); %number of outputs
nu = size(Bu,2); %number of inputs
nd = size(Bd,2); %number of disturbances

umax = 15*ones(nu,1); umin = 0*ones(nu,1); %input constraints
ymax = 26*ones(ny,1); ymin = 22*ones(ny,1); %output constraints

vmax = 20; vmin = -20;
xbmax = 20; xbmin = 0;

%% Controller Design (Setting-up MPC optimizer)

figure,
subplot(1,2,1),
stairs(refDist(2:3,:)'); %plot two of disturbance inputs
legend(ssM.disturbance(2:3));
xlabel('Step-Time: 20min');
ylabel('Disturbances (kW)');
subplot(1,2,2),
stairs(refDist(1,:)'); %plot one of disturbance inputs
legend(ssM.disturbance(1));
xlabel('Step-Time: 20min');
ylabel('Disturbances (C)');

savefig('disturbances.fig'); %save figure
disp('Click on Enter to continue with the Section 1.'); pause;
%% Section 1: tracking MPC

%define objective parameters
param_objective.R = eye(ny); %cost weight
param_objective.yref = [24 24 24]'; %reference outputs

%define constraints matrix
param_constraints.Mu = [eye(nu); -eye(nu)];
param_constraints.My = [eye(ny); -eye(ny)];
param_constraints.mu = [umax; -umin];
param_constraints.my = [ymax; -ymin];

%Determine the influence of the horizon length on the MPC scheme
param_simulation.horizon_lengths = [4, 15, 30, 50, 72, 90]; %arbitrary values of horizon length
tuning_horizon_length(ssM, param_constraints, param_objective, param_simulation, refDist);

%define simulation parameters
param_simulation.N = 72; %prediction horizon chosen - 1 day = 72*20min
param_simulation.T = size(refDist,2)-param_simulation.N; %simulation length in time-steps

%run tracking MPC with no night-setbacks and no variable cost
[xt, yt, ut, t] = trackingMPC(ssM, param_constraints, param_objective, param_simulation);

%display total economic cost
total_cost = 0.2*sum(ut(:));
sprintf('Total economic cost: $%d', total_cost)

disp('Click on Enter to continue with the Section 2.'); pause;
%% Section 2: economic MPC and soft constraints

%define other objective parameters
param_objective.c = 0.2; %fixed electricity price
param_objective.S = 50*eye(ny); %economic cost weight

%run economic MPC with no night-setbacks and no variable cost
[xt, yt, ut, t, total_cost] = economicMPC_sc(ssM, param_constraints, param_objective, param_simulation);

%display total economic cost
sprintf('Total economic cost: $%d', total_cost)

disp('Click on Enter to continue with the Section 3.'); pause;
%% Section 3: economic, soft constraints, and variable cost

%run economic MPC with variable cost, but no night-setbacks
[xt, yt, ut, t, total_cost] = economicMPC_sc_vc(ssM, param_constraints, param_objective, param_simulation);

%display total economic cost
sprintf('Total economic cost: $%d', total_cost)

disp('Click on Enter to continue with the Section 4.'); pause;
%% Section 4 : Night setbacks

%run economic MPC with variable cost and night-setbacks
[xt, yt, ut, t, total_cost] = economicMPC_sc_vc_sb(ssM, param_constraints, param_objective, param_simulation);

%display total economic cost
sprintf('Total economic cost: $%d', total_cost)

disp('Click on Enter to continue with the Section 5.'); pause;
%% Section 5 : Battery coupled with the building

%define constraints matrix
param_constraints.Mv = [eye(length(vmax)); -eye(length(vmin))];
param_constraints.Mxb = [eye(length(xbmax)); -eye(length(xbmin))];
param_constraints.mv = [vmax; -vmin];
param_constraints.mxb = [xbmax; -xbmin];

%run economic MPC with variable cost, night-setbacks and a battery storage
[xt, yt, ut, t, et, xbt, total_cost] = economicMPC_sc_vc_sb_battery(ssM, ssModel, param_constraints, param_objective, param_simulation);

%display total economic cost
sprintf('Total economic cost: $%d', total_cost)

%Analyse the influence of dissipation factor and storage capacity of the
%battery on total economic cost
param_simulation.alpha = [0.7 0.8 0.9]; %arbitrary values of dissipation factor
param_simulation.capacity = [10 20 30 40 50]; %arbitrary values of storage capacity
battery_usage(ssM, ssModel, param_constraints, param_objective, param_simulation);

disp('Finished.');