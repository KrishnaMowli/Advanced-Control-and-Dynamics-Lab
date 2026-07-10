% =========================================================================
% QUBE-Servo 2 Rotary Pendulum: Hybrid Swing-Up & Stabilization Control
% Hardware Platform: Quanser QUBE-Servo 2
% =========================================================================
clear; clc;

%% 1. System Parameters
% --- Electrical Motor Parameters ---
Rm = 8.4;       % Motor Armature Resistance (Ohms)
kt = 0.042;     % Current-Torque Constant (N-m/A)
km = 0.042;     % Back-EMF Constant (V-s/rad)

% --- Rotary Arm Parameters ---
mr = 0.095;     % Mass of horizontal rotary arm (kg)
r = 0.085;      % Total length of the arm (m)
Jr = mr*r^2/3;  % Moment of inertia about pivot (kg-m^2)
br = 1e-3;      % Viscous Damping Coefficient (N-m-s/rad)

% --- Pendulum Link Parameters ---
mp = 0.024;     % Mass of pendulum link (kg)
Lp = 0.129;     % Total length of pendulum (m)
l = Lp/2;       % Distance from pivot to pendulum center of mass (m)
Jp = mp*Lp^2/3; % Moment of inertia about pivot (kg-m^2)
bp = 5e-5;      % Viscous Damping Coefficient (N-m-s/rad)
g = 9.81;       % Gravity Constant (m/s^2)


%% 2. State-Space Model
% Determinant of the rigid-body inertia matrix (M)
det_M = (Jr * Jp) - (mp * l * r)^2;

% Equivalent damping (combining mechanical friction and electrical back-EMF)
breq = br + (kt * km) / Rm;

A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (mp^2 * l^2 * r * g) / det_M, -(Jp * br) / det_M, -(mp * l * r * bp) / det_M;
     0, (Jr * mp * g * l) / det_M, -(mp * l * r * br) / det_M, -(Jr * bp) / det_M];

B = [0;
     0;
     (Jp / det_M) * (km / Rm);
     (mp * l * r / det_M) * (km / Rm)];

C = [1, 0, 0, 0;
     0, 1, 0, 0];

D = [0; 
     0];

disp('--- State-Space Matrices ---')
disp('A ='); disp(A);
disp('B ='); disp(B);


%% 3. LQR Controller Design
% Optimal balancing matrix used once the pendulum reaches the upright cone.
Q = diag([10, 0, 0, 1]); 
R = 1;                

K_balance = lqr(A, B, Q, R);

disp('--- Linear Stabilization Gain Matrix ---')
disp('K_balance ='); disp(K_balance);


%% 4. Energy Calculation
Ep = 2 * mp * g * l * 1000; % Reference potential energy (mJ)
disp('--- Energy Metrics ---')
fprintf('Reference Swing-Up Target Energy: %0.2f mJ\n', Ep);


%% 5. Non-Linear Energy Swing-Up Parameter Setup
% [NOTE: This section sets up the parameters for the non-linear swing-up loop]
k_mu = 40.0;     % Non-linear swing-up acceleration gain (Tuned Parameter)
Max_Voltage = 5; % Armature saturation safety barrier (V)

fprintf('\n--- Swing-Up Controller Configurations ---\n');
fprintf('Proportional Swing-up Gain (k_mu): %0.1f\n', k_mu);


%% 6. Switching Logic / Control Law Documentation Template
% Below is the conceptual switching logic implemented when running this 
% code profile inside a real-time Simulink/QUARC loop:
%
% alpha_deg = alpha * (180/pi);
%
% if abs(alpha_deg) < 20
%     %% --- BALANCE MODE ---
%     % Apply linear state feedback stabilization
%     u = -K_balance * x;
% else
%     %% --- SWING-UP MODE ---
%     % Compute instant mechanical energy deviation
%     % E_current = 0.5 * Jp * alpha_dot^2 + mp * g * l * (cos(alpha) - 1);
%     % E_error = E_current - (Ep / 1000);
%     %
%     % Energy pumping control law (generates acceleration on rotary arm)
%     % u_swing = k_mu * E_error * sign(alpha_dot * cos(alpha));
%     % u = min(max(u_swing, -Max_Voltage), Max_Voltage);
% end