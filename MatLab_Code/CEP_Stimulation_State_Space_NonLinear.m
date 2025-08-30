% Motor and system parameters
J_m = 6.7e-4; % Rotor inertia (kg·m^2)
m2 = 0.190276; m3 = 0.198764; m4 = 0.306845; % Masses (kg)
r2 = 0.0682; r4 = 0.1451; % Center of gravity positions (m)
J2 = 407.655e-6; J3 = 3503.14e-6; J4 = 3673.87e-6; % Moments of inertia (kg·m^2)
a1 = 0.3; a2 = 0.09; a3 = 0.36; a4 = 0.26; % Link lengths (m)
p = 0.18; q = 0.0; % Fixed distances (m)
Kt = 0.76; % Motor torque constant (Nm/A)
Ke = 90 * (1/60); % Back-EMF constant (V·s/rad, converted from V·s/rpm)
L = 5.8e-3; % Inductance (H)
R = 0.8; % Resistance (Ohms)
n_max = 3000; % Max operating speed (rpm)
Ts = 10.2; % Continuous Stall Torque (Nm)
Tp = 19.7; % Peak Torque (Nm)
P = 3e3; % Continuous Output Power (W)

% Assumed kinematic values (for simplicity)
delta_theta3_theta2 = 0.8; % Assumed value for delta_theta3/delta_theta2
delta_theta4_theta2 = 0.6; % Assumed value for delta_theta4/delta_theta2
theta2 = 0.5; % Assume some value for theta2 (rad)
theta3 = 0.4; % Assume some value for theta3 (rad)

% Effective inertia calculation (with nonlinear terms)
J_eff = J_m + m2*r2^2 + J2 + m3*a2^2 + ...
        2*m3*a2*delta_theta3_theta2*(p*cos(theta2-theta3) + q*sin(theta2-theta3)) + ...
        (delta_theta3_theta2^2)*(m3*(p^2 + q^2) + J3) + ...
        (delta_theta4_theta2^2)*(m4*r4^2 + J4);

% Nonlinear terms for state-space matrices
sin_theta2_theta3 = sin(theta2 - theta3); 
cos_theta2_theta3 = cos(theta2 - theta3);

% Partial derivative of C with respect to theta2
dC_dtheta2 = -m3*a2*delta_theta3_theta2*(p*cos(theta2-theta3) + q*sin(theta2-theta3));

% State-space matrices (nonlinear model)
 A = [0, 1, 0;
     0, dC_dtheta2/J_eff, Kt/J_eff;
     0, -Ke/L, -R/L];
B = [0; 0; 1/L];
C = [1, 0, 0]; % Output is theta2
D = 0;

% Create state-space system
sys = ss(A, B, C, D);

% Simulation parameters
t = 0:0.01:5; % Time vector (5 seconds, 10ms step)
u = 10 * ones(size(t)); % Step input: constant voltage of 10V

% Simulate response
[y, t_out, x] = lsim(sys, u, t);

% Four-bar mechanism setup
a1 = 0.3; % Fixed link (ground) length
a2_link = 0.09; % Crank length
a3_link = 0.36; % Coupler length
a4_link = 0.26; % Output link length

% Initialize arrays for four-bar mechanism angles
theta3 = zeros(size(t)); % Initialize theta3 array
theta4 = zeros(size(t)); % Initialize theta4 array

% Solve for four-bar mechanism angles based on motor dynamics
for i = 1:length(t)
    % Solve for theta3 and theta4 using the loop closure equation
    eq1 = @(angles) real(a2_link*exp(1j*y(i)) + a1 - ...
                a3_link*exp(1j*angles(1)) - a4_link*exp(1j*angles(2)));
    eq2 = @(angles) imag(a2_link*exp(1j*y(i)) + a1 - ...
                a3_link*exp(1j*angles(1)) - a4_link*exp(1j*angles(2)));
    
    % Solve nonlinear equations for each time step
    angles = fsolve(@(angles) [eq1(angles); eq2(angles)], [0, 0]);
    theta3(i) = angles(1);
    theta4(i) = angles(2);
end

% Animation of Four-Bar Mechanism
figure;
hold on;
axis equal;
xlim([-0.5, 0.5]);
ylim([-0.5, 0.5]);
grid on;
title('Four-Bar Mechanism Animation');

% Initialize plot objects
link1 = plot([0, a1], [0, 0], 'k', 'LineWidth', 2); % Ground
link2 = plot([0, 0], [0, 0], 'r', 'LineWidth', 2); % Crank
link3 = plot([0, 0], [0, 0], 'g', 'LineWidth', 2); % Coupler
link4 = plot([0, 0], [0, 0], 'b', 'LineWidth', 2); % Output link

% Animate the motion
for i = 1:length(t)
    % Joint positions
    O = [0, 0]; % Ground joint
    A = [a2_link * cos(y(i)), a2_link * sin(y(i))]; % Crank-Coupler joint
    B = A + [a3_link * cos(theta3(i)), a3_link * sin(theta3(i))]; % Coupler-Output joint
    P = [a1, 0]; % Output joint
    
    % Update plot data
    set(link2, 'XData', [O(1), A(1)], 'YData', [O(2), A(2)]);
    set(link3, 'XData', [A(1), B(1)], 'YData', [A(2), B(2)]);
    set(link4, 'XData', [B(1), P(1)], 'YData', [B(2), P(2)]);
    
    pause(0.01); % Adjust the speed of the animation
end

disp('Animation complete.');

% Plot Previous Graphs
figure;

% Angular position of input crank
subplot(4, 2, 1);
plot(t_out, y);
title('$\theta_2$ (Angular Position)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\theta_2$ (rad)', 'Interpreter', 'latex');

% Angular velocity of input crank
theta2_dot = gradient(y, t_out(2) - t_out(1));
subplot(4, 2, 2);
plot(t_out, theta2_dot);
title('$\dot{\theta}_2$ (Angular Velocity)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\dot{\theta}_2$ (rad/s)', 'Interpreter', 'latex');

% Angular position of link 3
subplot(4, 2, 3);
plot(t_out, theta3);
title('$\theta_3$ (Angular Position of Link 3)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\theta_3$ (rad)', 'Interpreter', 'latex');

% Angular position of link 4
subplot(4, 2, 4);
plot(t_out, theta4);
title('$\theta_4$ (Angular Position of Link 4)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\theta_4$ (rad)', 'Interpreter', 'latex');

% Angular velocity of link 3
theta3_dot = gradient(theta3, t_out(2) - t_out(1));
subplot(4, 2, 5);
plot(t_out, theta3_dot);
title('$\dot{\theta}_3$ (Angular Velocity of Link 3)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\dot{\theta}_3$ (rad/s)', 'Interpreter', 'latex');

% Angular velocity of link 4
theta4_dot = gradient(theta4, t_out(2) - t_out(1));
subplot(4, 2, 6);
plot(t_out, theta4_dot);
title('$\dot{\theta}_4$ (Angular Velocity of Link 4)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('$\dot{\theta}_4$ (rad/s)', 'Interpreter', 'latex');

% Input voltage
subplot(4, 2, 7);
plot(t_out, 10 * ones(size(t_out)));
title('Input Voltage ($u$)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Voltage (V)');

% Motor torque (example calculation with placeholder)
motor_torque = Kt * theta2_dot; % Placeholder
subplot(4, 2, 8);
plot(t_out, motor_torque);
title('Motor Torque ($T$)', 'Interpreter', 'latex');
xlabel('Time (s)');
ylabel('Torque (Nm)');

% Display simulation complete message
disp('Simulation complete. Compare with mathematical model outputs.');
