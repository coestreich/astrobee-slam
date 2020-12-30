function qvec = RtoODE(p, tspan, q0, w0)

    x0 = [q0, w0]';
    sol = ode45(@(t,x) dynamics_step(t, x, p), tspan, x0);
    solpts = deval(sol, tspan)
    qvec = solpts(1:3,:);
    

end

