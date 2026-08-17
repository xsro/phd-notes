clear; clc; close all;

%% Simulation parameters
T  = 20;              % total simulation time
dt = 1e-4;            % time step
t  = 0:dt:T;
N  = length(t);

%% System and controller parameters
c   = 1.0;            % sliding surface parameter
eps = 0.5;            % period parameter epsilon
R   = 5.0;            % control gain

%% Initial conditions
x = zeros(1,N);
q = zeros(1,N);       % q = integral of x
sigma = zeros(1,N);
u = zeros(1,N);
b = zeros(1,N);

x(1) = 2.0;           % initial state
q(1) = 0.0;

%% Optional smoothing parameter
% If use_smooth = false, exact sign(sin(.)) is used.
% If use_smooth = true, tanh approximation is used to reduce numerical chattering.
use_smooth = false;
phi = 0.02;

%% Simulation loop
for k = 1:N-1

    %% Unknown sign-switching control direction
    % The controller does NOT use this information.
    if t(k) < 5
        b(k) = 1;
    elseif t(k) < 10
        b(k) = -1;
    elseif t(k) < 15
        b(k) = 1;
    else
        b(k) = -1;
    end

    %% Sliding variable
    sigma(k) = x(k) + c*q(k);

    %% Periodic sliding mode control
    s = sin(pi/eps * sigma(k));

    if use_smooth
        % Smooth approximation, useful for numerical simulation
        periodic_sign = tanh(s/phi);
    else
        % Ideal discontinuous controller
        periodic_sign = sign(s);
    end

    u(k) = R * periodic_sign;

    %% System dynamics
    x_dot = b(k) * u(k);
    q_dot = x(k);

    %% Euler integration
    x(k+1) = x(k) + dt*x_dot;
    q(k+1) = q(k) + dt*q_dot;
end

%% Store final values
b(N) = b(N-1);
sigma(N) = x(N) + c*q(N);
u(N) = u(N-1);

%% Plot results
figure('Color','w','Position',[100 100 900 700]);

subplot(4,1,1);
plot(t,x,'LineWidth',1.5);
grid on;
ylabel('$x(t)$','Interpreter','latex');
title('Periodic Sliding Mode Control under Unknown Sign-Switching Control Direction');

subplot(4,1,2);
plot(t,sigma,'LineWidth',1.5); hold on;
yline(0,'k--');
yline(eps,'r--');
yline(-eps,'r--');
yline(2*eps,'g--');
yline(-2*eps,'g--');
grid on;
ylabel('$\sigma(t)$','Interpreter','latex');
legend('$\sigma(t)$','periodic sliding surfaces','Interpreter','latex');

subplot(4,1,3);
plot(t,b,'LineWidth',1.5);
grid on;
ylabel('$b(t)$','Interpreter','latex');
ylim([-1.5 1.5]);

subplot(4,1,4);
plot(t,u,'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('$u(t)$','Interpreter','latex');