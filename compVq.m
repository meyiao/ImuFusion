function Vq = compVq(q, a)

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);

ax = a(1);
ay = a(2);
az = a(3);

v_dot_q1 = [2*ax*q1 - 2*ay*q4 + 2*az*q3;...
            2*ax*q4 + 2*ay*q1 - 2*az*q2;...
            2*ay*q2 - 2*ax*q3 + 2*az*q1];

v_dot_q2 = [2*ax*q2 + 2*ay*q3 + 2*az*q4;...
            2*ax*q3 - 2*ay*q2 - 2*az*q1;...
            2*ax*q4 + 2*ay*q1 - 2*az*q2];
 
v_dot_q3 = [2*ay*q2 - 2*ax*q3 + 2*az*q1;...
            2*ax*q2 + 2*ay*q3 + 2*az*q4;...
            2*ay*q4 - 2*ax*q1 - 2*az*q3];

v_dot_q4 = [2*az*q2 - 2*ay*q1 - 2*ax*q4;...
            2*ax*q1 - 2*ay*q4 + 2*az*q3;...
            2*ax*q2 + 2*ay*q3 + 2*az*q4];
        
Vq = [v_dot_q1, v_dot_q2, v_dot_q3, v_dot_q4];