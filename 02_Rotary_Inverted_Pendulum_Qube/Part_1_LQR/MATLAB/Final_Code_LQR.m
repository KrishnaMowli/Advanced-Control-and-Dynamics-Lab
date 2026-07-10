% =========================================================================
% QUBE-Servo 2 Rotary Pendulum: LQR Stabilization Control
% Hardware Platform: Quanser QUBE-Servo 2
% =========================================================================
clear; clc;

%% 1. System Parameters
% --- Electrical Motor Parameters ---
Rm = 8.4;       % Motor Armature Resistance (Ohms)
kt = 0.042;     % Current-Torque Constant (N-m/A)
km = 0.042;     % Back-EMF Constant (V-s/rad)

% --- Rotary Arm Parameters ---
mr = 0.095;     % Mass of the horizontal rotary arm (kg)
r = 0.085;      % Total length of the arm (m)
Jr = mr*r^2/3;  % Moment of inertia about pivot (kg-m^2)
br = 1e-3;      % Viscous Damping Coefficient (N-m-s/rad)

% --- Pendulum Link Parameters ---
mp = 0.024;     % Mass of the pendulum link (kg)
Lp = 0.129;     % Total length of the pendulum (m)
l = Lp/2;       % Distance from pivot to pendulum center of mass (m)
Jp = mp*Lp^2/3; % Moment of inertia about pivot (kg-m^2)
bp = 5e-5;      % Viscous Damping Coefficient (N-m-s/rad)
g = 9.81;       % Gravity Constant (m/s^2)


%% 2. State-Space Model
% Determinant of the rigid-body inertia matrix (M)
det_M = (Jr * Jp) - (mp * l * r)^2;

% Equivalent damping (combining mechanical friction and electrical back-EMF)
breq = br + (kt * km) / Rm;

% Linearized State-Space Matrices around the upright equilibrium [0; 0; 0; 0]
% State Vector x = [theta; alpha; theta_dot; alpha_dot]
%   theta     : Rotary arm angle
%   alpha     : Pendulum angle (0 is perfectly upright)
%   theta_dot : Rotary arm angular velocity
%   alpha_dot : Pendulum angular velocity

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

% Display State-Space Structure
disp('--- State-Space Matrices ---')
disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);


%% 3. LQR Controller Design
% The Linear Quadratic Regulator minimizes the cost function: 
% J = integral(x'*Q*x + u'*R*u) dt
%
% Tuning Rationale for Q and R:
% -------------------------------------------------------------------------
% Q (State Penalty Matrix): 
%   - Q(1,1) = 10 : Penalizes deviations in the rotary arm position (theta).
%                   Forces the arm to return to its home center position (0 rad).
%   - Q(2,2) = 0  : Relies entirely on the natural balance coupling and 
%                   velocity dampening for the pendulum angle transient.
%   - Q(3,3) = 0  : Allows the rotary arm velocity to vary freely during corrections.
%   - Q(4,4) = 1  : Penalizes high-frequency pendulum whipping/oscillations 
%                   (alpha_dot), maximizing transient balance smoothness.
%
% R (Control Effort Penalty):
%   - R = 1       : Normalizes the input penalty against the motor armature voltage
%                   limits to eliminate aggressive actuator saturation.

Q = diag([10, 0, 0, 1]); 
R = 1;                

% Compute the optimal state-feedback gain matrix
K = lqr(A, B, Q, R);

% Display Optimal Feedback Gains
disp('--- State-Feedback Gain Matrix ---')
disp('K ='); disp(K);


%% 4. Energy Calculation
% Calculates the potential energy boundary of the pendulum link.
% This represents the total work required to raise the link from the 
% downward suspended state directly to the upright balance point.

Ep = 2 * mp * g * l * 1000; % Potential energy boundary converted to mJ
Er = Ep;                    % Reference target energy matching potential limit

disp('--- Energy Metrics ---')
fprintf('Pendulum Potential Energy Boundary (Ep): %0.2f mJ\n', Ep);