function result = myfunc(p, tspan, x0)

    x0(5:7) = [p(7);p(8);p(9)];
    ODE_Sol = ode45(@(t,x) dynamics_step(t,x,p), tspan, x0);
    ODE_result = deval(ODE_Sol, tspan);
    result = ODE_result(1:3,:);

end

