function [dx] = ControlDynamics(t, x, u)
    m_1 = 23.902;
    m_2 = 3.88;
    L_1 = 0.45;
    L_2 = 0.45;
    lc_1 = 0.091;
    lc_2 = 0.048;
    I_zz1 = 1.266;
    I_zz2 = 0.093;
    g = 9.81;

    q_1 = x(1);
    q_2 = x(2);
    dq_1 = x(3);
    dq_2 = x(4);

    T = u(end);
    N = (numel(u) - 1)/2;
    t_points = linspace(0, T, N);
    tau_points = reshape(u(1:end-1), 2, [])';
    u = interp1(t_points, tau_points, t)';

    M = [m_2*L_1^2 + 2*m_2*cos(q_2)*L_1*lc_2 + m_1*lc_1^2 + m_2*lc_2^2 + I_zz1 + I_zz2, m_2*lc_2^2 + L_1*m_2*cos(q_2)*lc_2 + I_zz2;
                                   m_2*lc_2^2 + L_1*m_2*cos(q_2)*lc_2 + I_zz2,                         m_2*lc_2^2 + I_zz2];

    h = [g*cos(q_1)*(L_1*m_2 + lc_1*m_1) + g*lc_2*m_2*cos(q_1 + q_2) - L_1*lc_2*m_2*(sin(q_2)*dq_2^2 + 2*dq_1*sin(q_2)*dq_2);
                                                           L_1*lc_2*m_2*sin(q_2)*dq_1^2 + g*lc_2*m_2*cos(q_1 + q_2)];
    ddq = M\(u-h);

    dx = zeros(4,1);
    dx(1) = x(3);
    dx(2) = x(4);
    dx(3) = ddq(1);
    dx(4) = ddq(2);
end