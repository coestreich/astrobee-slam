function obj = objfunc(p, tspan, x_true, x0)

    x0(5:7) = p(7:9)';
    ODE_Sol = ode45(@(t,x) dynamics_step(t,x,p), tspan, x0);
    result = deval(ODE_Sol, tspan);
    obj = sum( ( (result(1:3,:) - x_true(1:3,:))'*(result(1:3,:) - x_true(1:3,:)) ));

end

