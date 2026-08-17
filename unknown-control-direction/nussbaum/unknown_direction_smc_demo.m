clear; clc; close all;

%% ============================================================
%  Unknown Control Direction Sliding Mode Control Demo
%
%  Plant:
%       x_dot = b*u
%
%  Unknown:
%       sign(b) is unknown
%
%  Sliding variable:
%       s = x
%
%  Controller:
%       u = -k*N(theta)*sgn(s)
%
%  Adaptation law:
%       theta_dot = |s|
%
%  Nussbaum function:
%       N(theta) = theta^2*cos(theta)
%
% ============================================================

%% True plant parameter
% The controller does NOT use the sign of b.
% You may try b = 1 or b = -1.
b = -1.0;          % unknown control direction

%% Controller parameter
k = 1.0;

%% Simulation parameters
T  = 30;           % total simulation time
dt = 1e-4;         % integration step
t  = 0:dt:T;
Nstep = length(t);

%% Initial conditions
x0 = 1.0;
theta0 = 0.0;

x     = zeros(1, Nstep);
theta = zeros(1, Nstep);
u     = zeros(1, Nstep);
s     = zeros(1, Nstep);
Nu    = zeros(1, Nstep);

x(1)     = x0;
theta(1) = theta0;

%% Sign function regularization
% For numerical simulation, the ideal sign function is replaced by tanh(s/eps).
% This avoids numerical chattering in discrete-time simulation.
eps_s = 1e-3;

%% Main simulation loop: fourth-order Runge-Kutta
for i = 1:Nstep-1

    z = [x(i); theta(i)];

    k1 = closed_loop_dynamics(z, b, k, eps_s);
    k2 = closed_loop_dynamics(z + 0.5*dt*k1, b, k, eps_s);
    k3 = closed_loop_dynamics(z + 0.5*dt*k2, b, k, eps_s);
    k4 = closed_loop_dynamics(z + dt*k3, b, k, eps_s);

    z_next = z + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    x(i+1)     = z_next(1);
    theta(i+1) = z_next(2);

end

%% Compute signals
for i = 1:Nstep
    s(i)  = x(i);
    Nu(i) = theta(i)^2*cos(theta(i));
    u(i)  = -k*Nu(i)*tanh(s(i)/eps_s);
end

%% Plot results
figure('Color','w');

subplot(4,1,1);
plot(t, x, 'LineWidth', 1.5);
grid on;
ylabel('x');
title('State x(t)');

subplot(4,1,2);
plot(t, s, 'LineWidth', 1.5);
grid on;
ylabel('s');
title('Sliding Variable s(t)=x(t)');

subplot(4,1,3);
plot(t, theta, 'LineWidth', 1.5);
grid on;
ylabel('\theta');
title('Adaptive Variable \theta(t)');

subplot(4,1,4);
plot(t, u, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('u');
title('Control Input u(t)');

figure('Color','w');
plot(t, Nu, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('N(\theta)');
title('Nussbaum Gain N(\theta)=\theta^2 cos(\theta)');

%% Display final values
fprintf('Final value of x      = %.6f\n', x(end));
fprintf('Final value of theta  = %.6f\n', theta(end));
fprintf('True value of b       = %.6f\n', b);


%% ============================================================
%  Closed-loop dynamics
% ============================================================
function dz = closed_loop_dynamics(z, b, k, eps_s)

    x     = z(1);
    theta = z(2);

    s = x;

    % Nussbaum function
    Ntheta = theta^2*cos(theta);

    % regularized sign function
    sigma = tanh(s/eps_s);

    % control law
    u = -k*Ntheta*sigma;

    % plant
    x_dot = b*u;

    % adaptation law
    theta_dot = abs(s);

    dz = [x_dot; theta_dot];

end