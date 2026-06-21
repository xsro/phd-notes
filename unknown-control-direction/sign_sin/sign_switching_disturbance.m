clear; clc; close all;

%% Simulation parameters
T  = 20;              % total simulation time
dt = 1e-4;            % time step
t  = 0:dt:T;
N  = length(t);

%% Controller parameters
c   = 1.0;            % sliding surface parameter
eps = 0.5;            % periodic sliding parameter
R   = 8.0;            % control gain, should be large enough

%% Initial conditions
x = zeros(1,N);
q = zeros(1,N);       % q = integral of x
sigma = zeros(1,N);
u = zeros(1,N);
b = zeros(1,N);
d = zeros(1,N);

x(1) = 2.0;
q(1) = 0.0;

%% Use smooth approximation or ideal sign
use_smooth = false;
phi = 0.02;           % boundary layer parameter if use_smooth = true

%% Simulation loop
for k = 1:N-1

    %% Unknown sign-switching control direction
    % The controller does NOT use b(k).
    if t(k) < 5
        b(k) = 1;
    elseif t(k) < 10
        b(k) = -1;
    elseif t(k) < 15
        b(k) = 1;
    else
        b(k) = -1;
    end

    %% Bounded disturbance
    % Example: sinusoidal disturbance + sign-type disturbance
    d(k) = 0.8*sin(3*t(k)) + 0.3*sign(sin(7*t(k)));

    %% Sliding variable
    sigma(k) = x(k) + c*q(k);

    %% Periodic sliding mode control
    s = sin(pi/eps * sigma(k));

    if use_smooth
        periodic_sign = tanh(s/phi);
    else
        periodic_sign = sign(s);
    end

    R = ;
    u(k) = R * periodic_sign;

    %% System dynamics with disturbance
    x_dot = b(k)*u(k) + d(k);
    q_dot = x(k);

    %% Euler integration
    x(k+1) = x(k) + dt*x_dot;
    q(k+1) = q(k) + dt*q_dot;
end

%% Store final values
b(N) = b(N-1);
d(N) = d(N-1);
sigma(N) = x(N) + c*q(N);
u(N) = u(N-1);

%% Plot results
figure('Color','w','Position',[100 100 900 800]);

subplot(5,1,1);
plot(t,x,'LineWidth',1.5);
grid on;
ylabel('$x(t)$','Interpreter','latex');
title('Periodic Sliding Mode Control with Sign-Switching Control Direction and Disturbance');

subplot(5,1,2);
plot(t,sigma,'LineWidth',1.5); hold on;
yline(0,'k--');
yline(eps,'r--');
yline(-eps,'r--');
yline(2*eps,'g--');
yline(-2*eps,'g--');
grid on;
ylabel('$\sigma(t)$','Interpreter','latex');
legend('$\sigma(t)$','periodic sliding surfaces','Interpreter','latex');

subplot(5,1,3);
plot(t,b,'LineWidth',1.5);
grid on;
ylabel('$b(t)$','Interpreter','latex');
ylim([-1.5 1.5]);

subplot(5,1,4);
plot(t,d,'LineWidth',1.2);
grid on;
ylabel('$d(t)$','Interpreter','latex');

subplot(5,1,5);
plot(t,u,'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('$u(t)$','Interpreter','latex');