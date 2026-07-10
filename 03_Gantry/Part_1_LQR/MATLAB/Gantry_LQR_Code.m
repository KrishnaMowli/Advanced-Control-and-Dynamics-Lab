% =========================================================================
% 2 DOF Gantry - LQR State-Feedback Stabilization Control
% Platform: Coupled Rotary Arm and Pendulum System
% =========================================================================
clear; clc;

%% 1. Define Physical Parameters
mp = 0.230;      % Pendulum mass (kg)
Jp = 0.3925;     % Pendulum inertia (kg*m^2)
Lr = 0.216;      % Rotary arm length (m)
Lp = 0.3365;     % Pendulum length (m)
Jr = 0.002;      % Arm inertia (kg*m^2)
Dr = 0.005;      % Arm damping
Dp = 0.002;      % Pendulum damping
Km = 0.0256;     % Motor torque constant
Kemf = 0.000275; % Back-EMF coefficient
g  = 9.81;       % Gravity

%% 2. Compute Linearized Model
% Determinant value evaluating coupled cross-inertial terms
det_val = (mp*Lr^2 + Jr)*(Jp + 0.25*mp*Lp^2) + (0.5*mp*Lp*Lr)^2;

%% 3. Construct State-Space Matrices
A = [ 0, 0, 1, 0;
      0, 0, 0, 1;
      0, ((0.5*mp*Lp)^2 * Lr * g)/det_val, ...
         -(Dr + Kemf)*(Jp + 0.25*mp*Lp^2)/det_val, ...
         -Dp*(0.5*mp*Lp*Lr)/det_val;
      0, -(0.5*mp*Lp*g)*(mp*Lr^2 + Jr)/det_val, ...
          (Dr + Kemf)*(0.5*mp*Lp*Lr)/det_val, ...
         -Dp*(mp*Lr^2 + Jr)/det_val ];

B = [ 0;
      0;
      Km*(Jp + 0.25*mp*Lp^2)/det_val;
     -Km*(0.5*mp*Lp*Lr)/det_val ];

C = [1, 0, 0, 0;
     0, 1, 0, 0];

D = [0; 0];

%% 4. Verify Controllability
% LQR optimization algorithms mathematically require the plant model (A,B) 
% to be fully controllable or stabilizable. This verification confirms that 
% the input actuator can govern all internal state trajectories.
Co = ctrb(A,B);

disp('--- Controllability Matrix Verification ---')
disp('Calculated Rank of Controllability Matrix (Co):')
disp(rank(Co))
disp('Required State Vector Dimension (n):')
disp(size(A,1))
fprintf('\n');

%% 5. Specify LQR Weighting Matrices
% Penalty matrices assigned to settle transient oscillations efficiently
Q = diag([250, 0, 5, 10]);   % State vector penalties
R = 1;                      % Control effort voltage penalty

%% 6. Compute LQR Gain
K_lqr = lqr(A, B, Q, R);

%% 7. Display Results
disp('--- Controller Synthesis Results ---')
disp('LQR Gain Matrix K:');
disp(K_lqr);

disp('State Vector Evolution Matrix A:');
disp(A);

disp('Control Input Coupling Matrix B:');
disp(B);

disp('Output Matrix C:');
disp(C);

disp('Direct Feedthrough Matrix D:');
disp(D);