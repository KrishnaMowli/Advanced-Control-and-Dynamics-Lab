function [Kp, Kv] = PV_Controller_Design(cartType, loadType)
% COMPUTE_PV_GAINS Calculate PV controller gains for Quanser IP01/IP02 plants.
%
%   [Kp, Kv] = COMPUTE_PV_GAINS(cartType, loadType) computes the proportional 
%   gain (Kp) and velocity gain (Kv) matching the student lab manual specs:
%   Percent Overshoot (PO) <= 10% and Peak Time (tp) = 0.15s.
%
%   Inputs (Optional):
%       cartType - String: 'IP01' or 'IP02' (Default: 'IP02')
%       loadType - String: 'NO_LOAD' or 'WEIGHT' (Default: 'NO_LOAD')
%
%   Outputs:
%       Kp       - Proportional Position Gain (V/m)
%       Kv       - Velocity Feedback Gain (V.s/m)


    if nargin < 1 || isempty(cartType)
        cartType = 'IP02'; % Default fallback plant type
    end
    if nargin < 2 || isempty(loadType)
        loadType = 'NO_LOAD'; % Default fallback load configuration
    end

    %% ====================================================================
    %  1. DESIGN SPECIFICATIONS
    %  ====================================================================
    PO = 10;          % Target Percent Overshoot (10%)
    tp = 0.150;       % Target Time to First Peak (150 ms)

    % Handout explicit damping specification (Section 7.2)
    zeta = 0.59; 
    
    % Calculate desired undamped natural frequency (wn)
    wn = pi / (tp * sqrt(1 - zeta^2));

    %% ====================================================================
    %  2. CONVERSION CONSTANTS & BASE MOTOR PARAMETERS
    %  ====================================================================
    K_OZ2N     = 0.2780139;
    K_IN2M     = 0.0254;
    K_RDPS2RPM = 60 / (2 * pi);

    Rm    = 2.6;                            % Armature Resistance (Ohm)
    Kt    = 1.088 * K_OZ2N * K_IN2M;        % Torque Constant (N.m/A)
    Km    = 0.804e-3 * K_RDPS2RPM;          % Back-EMF Constant (V.s/rad)
    Jm    = 5.523e-5 * K_OZ2N * K_IN2M;     % Rotor Inertia (kg.m^2)
    Kg    = 3.71;                           % Gearbox Ratio
    Eff_m = 1.0;                            % Motor Efficiency
    Eff_g = 1.0;                            % Gearbox Efficiency
    r_mp  = 0.5 / 2 * K_IN2M;               % Motor Pinion Radius (m)

    %% ====================================================================
    %  3. CONFIGURATION SELECTION (IP01 vs IP02)
    %  ====================================================================
    switch upper(cartType)
        case 'IP01'
            M   = 0.52;                     % Mass of IP01 Cart (kg)
            Beq = 0.9;                      % Viscous Damping Coefficient (N.s/m)
            
        case 'IP02'
            Mc2 = 0.57;                     % Base Mass of IP02 Cart (kg)
            Mw  = 0.37;                     % Weight Mass (kg)
            
            switch upper(loadType)
                case 'NO_LOAD'
                    M   = Mc2;
                    Beq = 4.3;
                case 'WEIGHT'
                    M   = Mc2 + Mw;
                    Beq = 5.4;
                otherwise
                    error('Invalid loadType for IP02. Use ''NO_LOAD'' or ''WEIGHT''.');
            end
            
        otherwise
            error('Invalid cartType. Use ''IP01'' or ''IP02''.');
    end

    %% ====================================================================
    %  4. DYNAMIC MODEL COEFFICIENTS (Appendix B.2)
    %  ====================================================================
    A = r_mp * Eff_g * Kg * Eff_m * Kt;
    B = Rm * M * r_mp^2 + Rm * Eff_g * Kg^2 * Jm;
    C = Eff_g * Kg^2 * Eff_m * Kt * Km + Beq * Rm * r_mp^2;

    %% ====================================================================
    %  5. CONTROLLER GAIN DERIVATION (Appendix C.3)
    %  ====================================================================
    Kp = (B * wn^2) / A;
    Kv = (2 * B * zeta * wn - C) / A;

    %% ====================================================================
    %  6. COMMAND LINE SUMMARY REPORT
    %  ====================================================================
    fprintf('\n========================================\n');
    fprintf('   PV CONTROLLER DESIGN REPORT (%s)\n', upper(cartType));
    if strcmpi(cartType, 'IP02')
        fprintf('   Load Configuration: %s\n', upper(loadType));
    end
    fprintf('========================================\n');
    fprintf('Target Performance Metrics:\n');
    fprintf('  - Percent Overshoot (PO): %0.1f%%\n', PO);
    fprintf('  - Damping Ratio (zeta):  %0.4f\n', zeta);
    fprintf('  - Natural Freq (wn):     %0.2f rad/s\n', wn);
    fprintf('Calculated System Dynamics:\n');
    fprintf('  - Combined Mass (M):     %0.3f kg\n', M);
    fprintf('  - B_eq Parameter:        %0.2f N.s/m\n', Beq);
    fprintf('Resulting Controller Gains:\n');
    fprintf('  --> Proportional Gain (Kp): %0.4f V/m\n', Kp);
    fprintf('  --> Velocity Gain (Kv):     %0.4f V.s/m\n', Kv);
    fprintf('========================================\n\n');


    %% ====================================================================
    %  6. PUBLISHABLE IMAGE GENERATION
    %  ===================================================================
    % Construct Closed-Loop Transfer Function: T(s) = (A*Kp) / (B*s^2 + (C + A*Kv)*s + A*Kp)
    num_cl = A * Kp;
    den_cl = [B, (C + A * Kv), A * Kp];
    sys_cl = tf(num_cl, den_cl);
    
    % Professional Figure Graphics Setup
    fontName = 'Helvetica';
    fontSize = 12;
    
    % --- FIGURE 1: CLOSED-LOOP STEP RESPONSE ---
    figure('Color', 'w', 'Name', 'Closed-Loop Step Response', 'Position', [100, 100, 600, 450]);
    t = 0:0.001:0.6; % Simulate up to 600ms to show transient decay clearly
    [y, t_out] = step(sys_cl, t);
    
    hold on;
    % Plot reference command profile
    plot([0, max(t)], [1, 1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', 'Setpoint Target');
    % Plot tracked plant output trajectory
    plot(t_out, y, 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5, 'DisplayName', 'Measured Cart Position');
    
    % Highlight Peak Time specification target
    plot(tp, 1 + (PO/100), 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Design Spec Point');
    
    hold off;
    grid on; box on;
    set(gca, 'FontName', fontName, 'FontSize', fontSize, 'LineWidth', 1.1);
    title(sprintf('Closed-Loop Position Step Response (%s - %s)', upper(cartType), upper(loadType)), ...
          'FontName', fontName, 'FontSize', fontSize + 2, 'FontWeight', 'bold');
    xlabel('Time (seconds)', 'FontName', fontName, 'FontSize', fontSize + 1);
    ylabel('Normalized Position (m)', 'FontName', fontName, 'FontSize', fontSize + 1);
    legend('Location', 'best', 'FontName', fontName, 'FontSize', fontSize - 1);
    axis([0 max(t) 0 1.25]);

    % --- FIGURE 2: S-PLANE POLE LOCATIONS ---
    figure('Color', 'w', 'Name', 'System Pole Locations', 'Position', [720, 100, 500, 450]);
    sys_poles = roots(den_cl);
    
    hold on;
    % Draw Imaginary and Real axis lines for reference
    xline(0, 'k-', 'LineWidth', 1.2);
    yline(0, 'k-', 'LineWidth', 1.2);
    
    % Plot dominant conjugate roots
    plot(real(sys_poles), imag(sys_poles), 'x', 'Color', [0.8500 0.3250 0.0980], ...
         'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Closed-Loop Poles');
    
    hold off;
    grid on; box on;
    set(gca, 'FontName', fontName, 'FontSize', fontSize, 'LineWidth', 1.1);
    title('Complex Conjugate Pole Locations (s-Plane)', 'FontName', fontName, ...
          'FontSize', fontSize + 2, 'FontWeight', 'bold');
    xlabel('Real Axis (s^{-1})', 'FontName', fontName, 'FontSize', fontSize + 1);
    ylabel('Imaginary Axis (rad/s)', 'FontName', fontName, 'FontSize', fontSize + 1);
    legend('Location', 'best', 'FontName', fontName, 'FontSize', fontSize - 1);
    
    % Add padding to axes dynamically for presentation value
    xDataLim = [min(real(sys_poles))*1.5, 0.5];
    yDataLim = [min(imag(sys_poles))*1.5, max(imag(sys_poles))*1.5];
    xlim(xDataLim); ylim(yDataLim);
end


