function cost = cost_fun(u)
    F_vec = u(1:end-1);
    T = u(end);
    N = (numel(u)-1)/2;
    dt = T/(N-1);

    tau_nodes = reshape(F_vec, 2, [])';
    % cost = sum(F_vec(1:end-1).^2)*dt; 
    torque_sq = tau_nodes(:,1).^2 + tau_nodes(:,2).^2;

    cost = dt * (0.5*torque_sq(1) ...
               + sum(torque_sq(2:end-1)) ...
               + 0.5*torque_sq(end));
end