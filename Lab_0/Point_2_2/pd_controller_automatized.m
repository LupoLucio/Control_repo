%% PD_CONTROLLER_AUTOMATIZED Design PD controller gains using frequency domain method
%  Computes PD gains (Kp, Kd) using lead compensation based on
%  desired phase margin and crossover frequency.
%
%  Syntax: [kp,ki,kd,tl,type] = pd_controller_automatized(wgc_p,delta_phi_rad,delta_K,wgc)
%
%  Inputs:
%    wgc_p             - Weight for time constant [unitless, 1-5]
%    delta_phi_rad     - Desired phase lead [radians]
%    delta_K           - Magnitude compensation factor [unitless]
%    wgc               - Crossover frequency [rad/s]
%
%  Outputs:
%    kp, kd            - Proportional and derivative gains
%    ki                - Integral gain (always 0 for PD)
%    tl                - Time constant for filter pole [seconds]
%    type              - Controller type label ('PD')
%
function [kp,ki,kd,tl,type] = pd_controller_automatized(wgc_p,delta_phi_rad,delta_K,wgc)
        
        % Input validation
        if wgc_p <= 0
            error('pd_controller_automatized:InvalidWgcP', 'Parameter wgc_p must be positive');
        end
        if wgc <= 0
            error('pd_controller_automatized:InvalidWgc', 'Crossover frequency wgc must be positive');
        end

        % Compute PD gains
        kp = delta_K * cos(delta_phi_rad);
        kd = (delta_K * sin(delta_phi_rad)) / wgc;
        ki = 0;  % No integral action for PD controller
        type = 'PD';
        
        % Compute filter time constant
        tl = 1 / (wgc_p * wgc); 
end