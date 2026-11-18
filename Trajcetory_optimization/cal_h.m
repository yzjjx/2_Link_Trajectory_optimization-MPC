syms m_1 m_2 L_1 lc_1 lc_2 I_zz1 I_zz2 g q_1 q_2 dq_1 dq_2 ddq_1 ddq_2

pi_param = [m_2*L_1^2 + m_1*lc_1^2 + I_zz1;
            L_1*lc_2*m_2;
            L_1*m_2 + lc_1*m_1;
            m_2*lc_2^2 + I_zz2;
            lc_2*m_2 ];

Y = [ddq_1, - sin(q_2)*dq_2^2 - 2*dq_1*sin(q_2)*dq_2 + 2*ddq_1*cos(q_2) + ddq_2*cos(q_2), g*cos(q_1), ddq_1 + ddq_2, g*cos(q_1 + q_2);
    0, sin(q_2)*dq_1^2 + ddq_1*cos(q_2),          0, ddq_1 + ddq_2, g*cos(q_1 + q_2)];

M = [m_2*L_1^2 + 2*m_2*cos(q_2)*L_1*lc_2 + m_1*lc_1^2 + m_2*lc_2^2 + I_zz1 + I_zz2, m_2*lc_2^2 + L_1*m_2*cos(q_2)*lc_2 + I_zz2;
                                m_2*lc_2^2 + L_1*m_2*cos(q_2)*lc_2 + I_zz2,                         m_2*lc_2^2 + I_zz2];

h = subs(Y*pi_param,ddq_1,0);
h = subs(h,ddq_2,0)