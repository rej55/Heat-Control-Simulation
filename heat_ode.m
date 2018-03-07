%% ODE function
function dx = heat_ode(x, A, B, R, F, r)
    u = -F*x;
    dx = A*x + B*u + R*r;
end