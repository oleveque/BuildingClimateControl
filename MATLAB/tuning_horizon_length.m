%%% Olivier Leveque & Maxime Maurin --  7 June 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tuning_horizon_length(ssM, param_constraints, param_objective, param_simulation, refDist)

horizons = param_simulation.horizon_lengths;
yref = param_objective.yref;

maxi = zeros(size(horizons));
times = zeros(size(horizons));
for i = 1:length(horizons)
    %define simulation parameters
    param_simulation.N = horizons(i); %prediction horizon
    param_simulation.T = size(refDist,2)-param_simulation.N; %simulation length in time-steps
    
    tic;
    %run tracking MPC with no night-setbacks and no variable cost
    [~, yt, ~, ~] = trackingMPC(ssM, param_constraints, param_objective, param_simulation);
    times(i) = toc; %measure running time
    maxi(i) = max(abs(max(yt,[],2)-yref)); %maximum error
end

figure,
subplot(1,2,1),
plot(horizons, maxi, '-O');
grid on;
xlabel('N: horizons');
ylabel('Error');
title('Influence of horizon length');

subplot(1,2,2),
plot(horizons, times, '-O');
grid on;
xlabel('N: horizons');
ylabel('Times');
title('Influence of horizon length');

savefig('tuning_horizon.fig'); %save the figure
end