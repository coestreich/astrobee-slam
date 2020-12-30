function q_prod = qmult(q1, q2)
    x1 = q1(1); y1 = q1(2); z1 = q1(3); w1 = q1(4);
    x2 = q2(1); y2 = q2(2); z2 = q2(3); w2 = q2(4);
    
    q_prod(1) = x1*w2 + y1*z2 - z1*y2 + w1*x2;
    q_prod(2) = -x1*z2 + y1*w2 + z1*x2 + w1*y2;
    q_prod(3) = x1*y2 - y1*x2 + z1*w2 + w1*z2;
    q_prod(4) = -x1*x2 - y1*y2 - z1*z2 + w1*w2;
    
end

