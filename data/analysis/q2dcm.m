function dcm = q2dcm(q)
    qw = q(4);
    qx = q(1);
    qy = q(2);
    qz = q(3);
    dcm = zeros(3,3);
    dcm(1,1) = qw^2 + qx^2 - qy^2 - qz^2;
    dcm(1,2) = 2*(qx*qy + qw*qz);
    dcm(1,3) = 2*(qx*qz - qw*qy);
    dcm(2,1) = 2*(qx*qy - qw*qz);
    dcm(2,2) = qw^2 - qx^2 + qy^2 - qz^2;
    dcm(2,3) = 2*(qy*qz + qw*qx);
    dcm(3,1) = 2*(qx*qz + qw*qy);
    dcm(3,2) = 2*(qy*qz - qw*qx);
    dcm(3,3) = qw^2 - qx^2 - qy^2 + qz^2;
end