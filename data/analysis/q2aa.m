function [aa] = q2aa(q)

    qx = q(1);
    qy = q(2);
    qz = q(3);
    qw = q(4);
    alpha = 2*atan2(sqrt(qx^2 + qy^2 + qz^2), qw);
    ex = qx/sqrt(qx^2 + qy^2 + qz^2);
    ey = qy/sqrt(qx^2 + qy^2 + qz^2);
    ez = qz/sqrt(qx^2 + qy^2 + qz^2);
    
    aa = [ex; ey; ez; alpha];
    

end

