%%% Olivier Leveque & Maxime Maurin --  7 June 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function battery_usage(ssM, ssModel, param_constraints, param_objective, param_simulation)

alpha = param_simulation.alpha;
capacity = param_simulation.capacity;

total_cost = zeros(length(alpha), length(capacity));
for i = 1:length(alpha)
    ssModel.A = alpha(i);
    
    for j = 1:length(capacity)
        
        %redefine constraints matrix
        xbmax = capacity(j); xbmin = 0;
        param_constraints.Mxb = [eye(length(xbmax)); -eye(length(xbmin))];
        param_constraints.mxb = [xbmax; -xbmin];
        
        %run economic MPC with variable cost, night-setbacks and a battery storage
        [~, ~, ~, ~, ~, ~, total_cost(i,j)] = economicMPC_sc_vc_sb_battery(ssM, ssModel, param_constraints, param_objective, param_simulation);
    end
end

figure,
surf(capacity, alpha, total_cost);
xlabel('Storage capacity')
ylabel('Dissipation factor of the battery')
zlabel('Total economic cost')
view([1, 1, 1]);

savefig('battery_usage.fig'); %save the figure
end