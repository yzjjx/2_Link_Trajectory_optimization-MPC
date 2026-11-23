function cost = cost_fun(w,q_ref)

    N = numel(w)/6;
    tau1 = w(1:N);
    tau2 = w(N+1:2*N);
    q1 = w(2*N+1:3*N);
    q2 = w(3*N+1:4*N);
    dq1 = w(4*N+1:5*N);
    dq2 = w(5*N+1:6*N);
    q1_err = q_ref(1)-q1;
    q2_err = q_ref(2)-q2;
    % cost = sum(F_vec(1:end-1).^2)*dt; 

    cost = sum(q1_err.^2 + q2_err.^2)+ 1e-4 * sum(diff(tau1).^2 + diff(tau2).^2);
end