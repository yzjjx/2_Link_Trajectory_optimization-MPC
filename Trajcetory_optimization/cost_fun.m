function cost = cost_fun(x, params)
    F_vec = x(1:end-1);
    T = x(end);
    N = numel(x)-1;
    dt = T/(N-1);

    % cost = sum(F_vec(1:end-1).^2)*dt; 
    cost = 0.5*F_vec(1)^2 + sum(F_vec(2:end-1).^2)*dt + 0.5*F_vec(end)^2;
end