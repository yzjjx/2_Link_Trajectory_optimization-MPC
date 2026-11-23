function cost = cost_fun(w)
    T = w(end);
    N = (numel(w)-1)/6;
    dt = T/(N-1);
    tau1 = w(1:N);
    tau2 = w(N+1:2*N);
    % cost = sum(F_vec(1:end-1).^2)*dt; 
    torque_sq = tau1.^2 + tau2.^2;

    cost = dt * (0.5*torque_sq(1) ...
               + sum(torque_sq(2:end-1)) ...
               + 0.5*torque_sq(end));
end