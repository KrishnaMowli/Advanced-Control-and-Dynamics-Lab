%Model Parameters
%Model parameters:

%% Amplifier Configuration
% Amplifier gain used for yaw and pitch axes.
K_AMP = 3;
% Amplifier Maximum Output Voltage (V)
VMAX_AMP = 24;
% Digital-to-Analog Maximum Voltage (V): set to 10 for Q4/Q8 cards
VMAX_DAC = 10;
%
%% Filter and Rate Limiter Settings
% Specifications of a second-order low-pass filter
wcf = 2 * pi * 20; % filter cutting frequency
zetaf = 0.6;        % filter damping ratio
%
% Maximum Rate of Desired Position (rad/s)
% Note: This is for both the program and joystick commands.
CMD_RATE_LIMIT = 60 * pi/180; % 60 deg/s converted to rad/s
%
%% Joystick Settings
% Joystick input X sensitivity used for roll (deg/s/V)
K_JOYSTICK_X = -25;
% Joystick input Y sensitivity used for pitch (deg/s/V)
K_JOYSTICK_Y = 25;
% Pitch integrator saturation of joystick (deg)
INT_JOYSTICK_SAT_LOWER = -10;
INT_JOYSTICK_SAT_UPPER = 10;
% Deadzone of joystick: set input ranging from -DZ to +DZ to 0 (V)
JOYSTICK_X_DZ = 0.25;
JOYSTICK_Y_DZ = 0.25;
%
%
%% Set the model parameters of the 3DOF HOVER.
% These parameters are used for model representation and controller design.
[ Kt, Kf, L, Jy, Jp, Jr, g, K_EC_Y , K_EC_P , K_EC_R ] = config_hover();
%
% For the following state vector: X = [ theta; psi; theta_dot; psi_dot]
% Initialization the state-Space representation of the open-loop System



A= [0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0];
B =[0,0,0,0;
    0,0,0,0;
    0,0,0,0;
    -Kt/Jy,-Kt/Jy,Kt/Jy,Kt/Jy;
    Kf*L/Jp,-Kf*L/Jp,0,0;
    0,0,Kf*L/Jr,-Kf*L/Jr];


C= [1,0,0,0,0,0;
    0,1,0,0,0,0;
    0,0,1,0,0,0];
D= [0,0,0,0;
    0,0,0,0;
    0,0,0,0];


Q= diag([350;350;350;25;25;25]);
R= 0.01*eye(4);

K=lqr(A,B,Q,R);

disp(A)
disp(B)
disp(C)
disp(D)
disp('K =')
disp(K)

%Q= diag([450;450;450;50;50;50]);
%Q= diag([500;500;500;50;50;50]);

%Q= diag([450;450;450;25;25;25]);
%Q= diag([350;350;350;25;25;25]);
