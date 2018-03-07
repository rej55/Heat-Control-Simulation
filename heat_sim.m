%% Clear
clear
close

%% Set model
% Calculation area
lx = 0.1;
ly = 0.1;
% Number of meshes
nx = 50;
ny = 50;
% Number of input
nu = 2;
% Mesh width
dx = lx/nx;
dy = ly/ny;
% Thermal diffusion coefficient
kappa = 13.9 * 10^-6;
% Set matrices
A = zeros(nx*ny);
B = zeros(nx*ny, nu);
for iy = 1:ny
    for ix = 1:nx
        id = ix + (iy-1)*nx;
        iw = id - 1;
        ie = id + 1;
        is = id - nx;
        in = id + nx;
        
        % Set boundary condition  (Neumann)
        if(ix ~= 1)
            A(id, iw) = 1/(dx^2);
            A(id, id) = A(id, id) - A(id, iw);
        end
        if(ix ~= nx)
            A(id, ie) = 1/(dx^2);
            A(id, id) = A(id, id) - A(id, ie);
        end
        if(iy ~= 1)
            A(id, is) = 1/(dy^2);
            A(id, id) = A(id, id) - A(id, is);
        end
        if(iy ~= ny)
            A(id, in) = 1/(dy^2);
            A(id, id) = A(id, id) - A(id, in);
        end
        
        % Set input ports
        if(ismember(iy, [1:5]))
            if(ismember(ix, [6:20]))
                B(id, 1) = 1;
            elseif(ismember(ix, [31:45]))
                B(id, 2) = 1;
            end
        end
    end
end
A = kappa * A;

% Set matrices of servo system
I = eye(nx*ny);
As = [A zeros(nx*ny, 1);-(1/(nx*ny))*ones(1,nx*ny) 0];
Bs = [B;zeros(1, nu)];
Rs = [zeros(nx*ny, 1);1];
Zs = [Bs Rs];

%% Design stabilization controller
F = lqr(As, Bs, diag([0.01*ones(1, nx*ny) 1]), eye(nu));

%% Control simulation
% Initial temperature
T0 = 250;
% Reference temperature
Tr = 300;
% Initial value setting
x0 = T0 * ones(nx*ny, 1) + 1*randn(nx*ny, 1);
% Reference value setting
r = Tr;
% Initial value of servo system
xs0 = [x0;r-mean(x0)];
% Solve ODE
[t, xs] = ode45(@(t, x) heat_ode(x, As, Bs, Rs, F, r), [0:1/30:300], xs0);
% Pick temperature
x = xs(:, 1:nx*ny);

%% Visualization
for i = 1:10:length(t)
    T = reshape(x(i,:), [nx, ny])';
    imagesc(T);
    title(['t = ' num2str(t(i))]);
    caxis([200 450])
    colorbar
    colormap hot
    ax = gca;
    ax.FontSize = 18;
    drawnow;
    F(i) = getframe(gcf);
    pause(0.01)
end


%% ÉrÉfÉIèëÇ´çûÇ›
%{
v = VideoWriter('heat_sim.mp4', 'MPEG-4');
open(v);
for i = 1:10:length(t)
writeVideo(v, F(i));
end
close(v);
%}
