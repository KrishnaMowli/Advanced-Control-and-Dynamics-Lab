% =========================================================================
% 3DOF Hover - LQG (LQR + Kalman Filter) Synthesis Script
% Platform: Quanser 3DOF Hover System
% =========================================================================
clear; clc;

%% 1. Hardware & System Configuration Settings
% --- Amplifier Configuration ---
K_AMP = 3;              % Amplifier gain used for yaw and pitch axes
VMAX_AMP = 24;          % Amplifier Maximum Output Voltage (V)
VMAX_DAC = 10;          % Digital-to-Analog Maximum Voltage (V)

% --- Filter and Rate Limiter Settings ---
wcf = 2 * pi * 20;      % Second-order low-pass filter cutoff frequency (rad/s)
zetaf = 0.6;            % Filter damping ratio
CMD_RATE_LIMIT = 60 * pi/180; % Maximum Rate of Desired Position (rad/s)

% --- Joystick Sensitivity Settings ---
K_JOYSTICK_X = -25;     % Joystick input X sensitivity (roll: deg/s/V)
K_JOYSTICK_Y = 25;      % Joystick input Y sensitivity (pitch: deg/s/V)
INT_JOYSTICK_SAT_LOWER = -10;
INT_JOYSTICK_SAT_UPPER = 10;
JOYSTICK_X_DZ = 0.25;   % Deadzone boundary for joystick X axis (V)
JOYSTICK_Y_DZ = 0.25;   % Deadzone boundary for joystick Y axis (V)

% --- Load Plant Rigid Body Parameters ---
% Loads physical constants: Thrust constants, frame lengths, moments of inertia.
[ Kt, Kf, L, Jy, Jp, Jr, g, K_EC_Y , K_EC_P , K_EC_R ] = config_hover();


%% 2. Construct State-Space Matrices
% State Vector: X = [theta; psi; phi; theta_dot; psi_dot; phi_dot]
%   theta: Pitch,  psi: Yaw,  phi: Roll  (and their respective derivatives)
A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0];
 
B = [0, 0, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0;
     -Kt/Jy, -Kt/Jy,  Kt/Jy,  Kt/Jy;
      Kf*L/Jp, -Kf*L/Jp, 0,      0;
      0,       0,       Kf*L/Jr, -Kf*L/Jr];

C = [1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];
 
D = [0, 0, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0];


%% 3. Controllability Verification & LQR Design
% Calculate the Controllability Matrix (Co) to verify that the plant's 4-motor 
% input array can fully actuate and stabilize all 6 rigid-body states.
Co = ctrb(A, B);

disp('--- Controllability Verification ---')
disp('Calculated Rank of Controllability Matrix (Co):')
disp(rank(Co))
disp('Expected Full State Dimension (n):')
disp(size(A, 1))
fprintf('\n');

% --- LQR Penalty Weight Matrices ---
Q = diag([450; 450; 450; 200; 100; 100]);  % State deviation weights
R = 0.01 * eye(4);                         % Motor voltage control penalty

% Compute the optimal infinite-horizon state-feedback gain matrix
K = lqr(A, B, Q, R);


%% 4. Observability Verification & Kalman Filter (LQE) Design
% Calculate the Observability Matrix (Ob) to verify that measuring only the 3 
% spatial positions (theta, psi, phi) is mathematically sufficient to 
% dynamically reconstruct the remaining 3 velocity states.
Ob = obsv(A, C);

disp('--- Observability Verification ---')
disp('Calculated Rank of Observability Matrix (Ob):')
disp(rank(Ob))
disp('Expected Full State Dimension (n):')
disp(size(A, 1))
fprintf('\n');

% --- Stochastic Noise Covariance Matrices ---
N1 = 0.01 * eye(6);    % Process noise covariance matrix
CN2 = 0.05 * eye(3);   % Sensor/Measurement noise covariance matrix
NU = 1 * eye(6);       % Input noise scaling matrix

% Compute the optimal steady-state Kalman estimator gain matrix
KF = lqe(A, NU, C, N1, CN2);


%% 5. Assemble Integrated Estimator System
% Construct the complete observer system architecture
sysKF = ss((A - KF * C), [B, KF], N1, 0);
sysKF.a = A - KF * C;
sysKF.b = [B, KF];
sysKF.c = eye(6);
sysKF.d = zeros(6, 7);


%% 6. Display Synthesis Results
disp('===========================================')
disp('        LQG CONTROLLER SYNTHESIS')
disp('===========================================')
disp('State Transition Matrix A:'); disp(A);
disp('Input Distribution Matrix B:'); disp(B);
disp('Output Measurement Matrix C:'); disp(C);
disp('Direct Feedthrough Matrix D:'); disp(D);
disp('Optimal LQR Regulator Gain Matrix K:'); disp(K);
disp('Optimal Kalman Filter Gain Matrix KF:'); disp(KF);
disp('===========================================')