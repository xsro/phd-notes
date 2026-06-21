function dx = APSM_ode(t, x, params)
% 状态向量: x = [x1; x2; R]
% 提取状态
x1 = x(1);
x2 = x(2);
R  = x(3);

% 提取参数
b = params.b(t);
d = params.d(t);
alpha = params.alpha;
epsilon0 = params.epsilon0;
lambda = params.lambda;
r0 = params.r0;
mu = params.mu;
gamma = params.gamma;

% 计算时变 epsilon
epsilon = epsilon0 * exp(-lambda * t);

% 滑模面
s = x2 + alpha * x1;

% 周期切换函数 (避免在切换点处数值问题)
if abs(s) < 1e-12
    phi = 0;   % 避免 sign(0) 产生零
else
    arg = pi * s / epsilon;
    % 为防止溢出, 先取 sin 再取符号
    phi = sign(sin(arg));
end

% 控制律
u = R * phi;

% 自适应律
dR = mu * abs(s) - gamma * (R - r0);

% 系统动态
dx1 = x2;
dx2 = b * u + d;

% 返回导数
dx = [dx1; dx2; dR];
end