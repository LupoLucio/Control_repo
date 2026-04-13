%% PI_CONTROLLER_AUTOMATIZED Design PI controller gains using frequency domain method
%  Computes PI gains (Kp, Ki) using lag compensation based on
%  desired phase margin and crossover frequency.
%
%  Syntax: [kp,ki,kd,tl,type] = pi_controller_automatized(wgc_p,delta_phi_rad,delta_K,wgc)
%
%  Inputs:
%    wgc_p             - Weight for time constant [unitless, 1-5]
%    delta_phi_rad     - Desired phase compensation [radians]
%    delta_K           - Magnitude compensation factor [unitless]
%    wgc               - Crossover frequency [rad/s]
%
%  Outputs:
%    kp, ki            - Proportional and integral gains
%    kd                - Derivative gain (always 0 for PI)
%    tl                - Time constant for filter pole [seconds]
%    type              - Controller type label ('PI')
%
function [kp,ki,kd,tl,type] = pi_controller_automatized(wgc_p,delta_phi_rad,delta_K,wgc)
        
        % Input validation
        if wgc_p <= 0
            error('pi_controller_automatized:InvalidWgcP', 'Parameter wgc_p must be positive');
        end
        if wgc <= 0
            error('pi_controller_automatized:InvalidWgc', 'Crossover frequency wgc must be positive');
        end

        type = 'PI';
        
        % Compute PI gains
        kp = delta_K * cos(delta_phi_rad);
        ki = -wgc * delta_K * sin(delta_phi_rad);
        kd = 0;  % No derivative action for PI controller
        
        % Compute filter time constant
        tl = 1 / (wgc_p * wgc); 
end