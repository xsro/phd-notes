clear; clc; close all;

%% ============================================================
% Unknown Control Direction Sliding Mode Control
% for Disturbed Double Integrator
%
% Plant:
%   x1_dot = x2
%   x2_dot = b*u + d(t)
%
% Unknown:
%   sign(b) is unknown
%
% Sliding variable:
%   s = x2 + lambda*x1
%
% Controller:
%   u = -k*N(theta)*sat(s/eps)
%
% Adaptation law:
%   theta_dot = |s|
%
% Nussbaum function:
%   N(theta) = theta^2*cos(theta)
%
% ============================================================

%% True plant parameter
% The controller does not use the sign of b.
% Try b = 1, b = -1, b = 2, b = -2.
b = -1.0;

%% Control parameters
lambda = 2.0;      % sliding surface parameter
k      = 2.0;      % control gain
eps_s  = 0.02;     % boundary layer thickness

%% Simulation parameters
T  = 30;
dt = 1e-4;
t  = 0:dt:T;
Nstep = length(t);

%% Initial conditions
x1_0    = 1.0;
x2_0    = 0.0;
theta_0 = 0.0;

x1    = zeros(1,Nstep);
x2    = zeros(1,Nstep);
theta = zeros(1,Nstep);
s     = zeros(1,Nstep);
u     = zeros(1,Nstep);
d     = zeros(1,Nstep);
Ntheta = zeros(1,Nstep);

x1(1)    = x1_0;
x2(1)    = x2_0;
theta(1) = theta_0;

%% Main simulation loop: RK4 integration
for i = 1:Nstep-1

    z = [x1(i); x2(i); theta(i)];

    ti = t(i);

    k1 = closed_loop_dynamics(ti, z, b, lambda, k, eps_s);
    k2 = closed_loop_dynamics(ti + 0.5*dt, z + 0.5*dt*k1, b, lambda, k, eps_s);
    k3 = closed_loop_dynamics(ti + 0.5*dt, z + 0.5*dt*k2, b, lambda, k, eps_s);
    k4 = closed_loop_dynamics(ti + dt, z + dt*k3, b, lambda, k, eps_s);

    z_next = z + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    x1(i+1)    = z_next(1);
    x2(i+1)    = z_next(2);
    theta(i+1) = z_next(3);

end

%% Compute signals
for i = 1:Nstep
    s(i) = x2(i) + lambda*x1(i);
    d(i) = disturbance(t(i));
    Ntheta(i) = theta(i)^2*cos(theta(i));
    u(i) = -k*Ntheta(i)*sat(s(i)/eps_s);
end

%% Plot results
figure('Color','w');

subplot(5,1,1);
plot(t,x1,'LineWidth',1.5);
grid on;
ylabel('x_1');
title('Position State x_1(t)');

subplot(5,1,2);
plot(t,x2,'LineWidth',1.5);
grid on;
ylabel('x_2');
title('Velocity State x_2(t)');

subplot(5,1,3);
plot(t,s,'LineWidth',1.5);
grid on;
ylabel('s');
title('Sliding Variable s=x_2+\lambda x_1');

subplot(5,1,4);
plot(t,u,'LineWidth',1.5);
grid on;
ylabel('u');
title('Control Input u(t)');

subplot(5,1,5);
plot(t,theta,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('\theta');
title('Adaptive Variable \theta(t)');

figure('Color','w');

subplot(2,1,1);
plot(t,Ntheta,'LineWidth',1.5);
grid on;
ylabel('N(\theta)');
title('Nussbaum Gain N(\theta)=\theta^2 cos(\theta)');

subplot(2,1,2);
plot(t,d,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('d(t)');
title('External Disturbance');

%% Print final values
fprintf('Final x1     = %.6f\n', x1(end));
fprintf('Final x2     = %.6f\n', x2(end));
fprintf('Final s      = %.6f\n', s(end));
fprintf('Final theta  = %.6f\n', theta(end));
fprintf('True b       = %.6f\n', b);

%% ============================================================
% Closed-loop dynamics
% ============================================================
function dz = closed_loop_dynamics(t, z, b, lambda, k, eps_s)

    x1    = z(1);
    x2    = z(2);
    theta = z(3);

    % sliding variable
    s = x2 + lambda*x1;

    % Nussbaum function
    Ntheta = theta^2*cos(theta);

    % control law
    u = -k*Ntheta*sat(s/eps_s);

    % disturbance
    d = disturbance(t);

    % plant dynamics
    x1_dot = x2;
    x2_dot = b*u + d;

    % adaptation law
    theta_dot = abs(s);

    dz = [x1_dot; x2_dot; theta_dot];

end

%% ============================================================
% Saturation function
% ============================================================
function y = sat(x)

    if x > 1
        y = 1;
    elseif x < -1
        y = -1;
    else
        y = x;
    end

end

%% ============================================================
% Bounded disturbance
% ============================================================
function d = disturbance(t)

    d = 0.2*sin(2*t) + 0.1*cos(0.5*t);

end