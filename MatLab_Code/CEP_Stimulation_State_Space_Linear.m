%
% Parameters
J_m = 6.7e-4; % Rotor inertia (kg·m^2)
m2 = 0.190276; m3 = 0.198764; m4 = 0.306845; % Masses (kg)
r2 = 0.0682; r4 = 0.1451; % Center of gravity positions (m)
J2 = 0.0004077; J3 = 0.003503; J4 = 0.003674; % Moments of inertia (kg·m^2)
a2 = 0.09; p = 0.18; % Link lengths (m)
Kt = 0.76; % Motor torque constant (Nm/A)
Ke = 0.09; % Back-EMF constant (V·s/rad)
L = 5.8e-3; % Inductance (H)
R = 0.8; % Resistance (Ohms)

% Effective inertia calculation
delta_theta3_theta2 = 0.8; % Assumed
delta_theta4_theta2 = 0.6; % Assumed
J_eff = J_m + m2*r2^2 + J2 + m3*a2^2 + ...
        2*m3*a2*delta_theta3_theta2*p + ...
        (delta_theta3_theta2^2)*(m3*p^2 + J3) + ...
        (delta_theta4_theta2^2)*(m4*r4^2 + J4);

% State-space matrices (linearized)
A = [0, 1, 0;
     0, 0, Kt/J_eff;
     0, -Ke/L, -R/L];
B = [0; 0; 1/L];
C = [1, 0, 0]; % Output is theta2
D = 0;

% Create state-space system
sys = ss(A, B, C, D);

% Simulation parameters
t = 0:0.001:5; % Time vector (5 seconds, 1ms step)
u = 10 * ones(size(t)); % Step input: constant voltage of 10V

% Simulate response
[y, t_out, x] = lsim(sys, u, t);

% Plot results
figure;

% Create 4x2 grid for subplots
subplot(4, 2, 1);
plot(t_out, y);
title('$\theta_2$ (Angular Position)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\theta_2$ (rad)', 'Interpreter', 'latex');

subplot(4, 2, 2);
plot(t_out, x(:, 2));
title('$\dot{\theta}_2$ (Angular Velocity)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\dot{\theta}_2$ (rad/s)', 'Interpreter', 'latex');

subplot(4, 2, 3);
plot(t_out, x(:, 3));
title('Motor Current ($i_m$)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Current (A)');

% Additional plots
subplot(4, 2, 4);
plot(t_out, u);
title('Input Voltage ($u$)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Voltage (V)');

subplot(4, 2, 5);
plot(t_out, Kt * x(:, 3));
title('Motor Torque ($T$)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Torque (Nm)');

subplot(4, 2, 6);
plot(t_out(1:end-1), diff(x(:, 3))/diff(t_out(1:2)));
title('Motor Current Derivative ($d(i_m)/dt$)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$d(i_m)/dt$ (A/s)', 'Interpreter', 'latex');

% Plot angular position for other links (delta_theta3_theta2, delta_theta4_theta2 assumptions)
subplot(4, 2, 7);
theta3 = delta_theta3_theta2 * y; % Example of link 3 angular position
plot(t_out, theta3);
title('$\theta_3$ (Angular Position of Link 3)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\theta_3$ (rad)', 'Interpreter', 'latex');

% Plot angular velocity of link 3 (assuming same scaling)
subplot(4, 2, 8);
theta3_dot = delta_theta3_theta2 * x(:, 2); % Example of link 3 angular velocity
plot(t_out, theta3_dot);
title('$\dot{\theta}_3$ (Angular Velocity of Link 3)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\dot{\theta}_3$ (rad/s)', 'Interpreter', 'latex');

% Display simulation complete message
disp('Simulation complete. Compare with mathematical model outputs.');
