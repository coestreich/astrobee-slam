function xdot = dynamics_step(t, x, p)

    J = [p(1), p(4), p(5); ...
         p(4), p(2), p(6); ...
         p(5), p(6), p(3)];
    
        
    q = x(1:4);
    w = x(5:7);
    M = [0, 0, 0]';  % assuming no external torques on target
    
    H = [q(4), q(3), -q(2), -q(1); ...
         -q(3), q(4), q(1), -q(2); ...
         q(2), -q(1), q(4), -q(3)];
    
    qdot = 0.5*H'*w;
    
    %disp(q);
    %disp(w);

    wdot = J \ (M - cross(w, J*w));
    
    xdot = zeros(7,1);
    xdot(1:4) = qdot;
    xdot(5:7) = wdot;
    
end