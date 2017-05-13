%%% Olivier Leveque & Maxime Maurin --  7 June 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%INPUT:
%  t - simulation time-step
%  N - Prediction Horizon of your MPC controller

%OUTPUTS:
% pred - prediction of the disturbance input, over the MPC prediction horizon
% cp - shifted price of electricity consumption over the MPC prediction horizon, at time step t
% sb - shifted comfort constraint off-sets over the MPC prediction horizon, at time step t


function [pred, cp, sb] = shiftPred(t, N)

load building.mat;


%% Disturbance Prediction
pred = refDist(:,t:t+N-1);


%% Variable Price Prediction 
cp = zeros(1,N);
for i = 1:N
    if and(10<mod((t+i)/3, 24), mod((t+i)/3, 24)<=16)
        cp(i) = 0.2;
    else
        cp(i) = 0.04;
    end
end

%% Night-Setback Prediction
ny = size(ssM.C,1); %number of outputs
ymax_sb = 4;
ymin_sb = -4;

sb = zeros(2*ny,N);
for i = 1:N
    if and(8*3<mod((t+i), 24*3), mod((t+i), 24*3)<=18*3)
        sb(:,i) = zeros(2*ny,1);
    else
        sb(:,i) = kron([ymax_sb; -ymin_sb], ones(ny,1));
    end
end

end

