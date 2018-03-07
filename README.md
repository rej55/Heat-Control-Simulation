# Heat-Control-Simulation
This program is a 2D heat control simulator based on boundary control of heat difffusion equation.

$$
\frac{\partial T}{\partial t} = \kappa\left(\frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2}\right)
$$

## Simulation Flow
1. Space discretization of heat difffusion equation
2. Set matrices of servo system
3. Design feedback matrix by LQR method
4. Solve ODE
5. Visualization

## License
MIT License