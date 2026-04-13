%% PID_CONTROLLER_AUTOMATIZED Design PID controller gains using frequency domain method
%  Computes PID gains (Kp, Ki, Kd) using lead-lag compensation based on
%  desired phase margin and crossover frequency.
%
%  Syntax: [kp,ki,kd,tl,type] = pid_controller_automatized(alfa,wgc_p,delta_phi_rad,delta_K,wgc)
%
%  Inputs:
%    alfa              - Ratio parameter for PID tuning (typically 4-10)
%    wgc_p             - Weight for time constant (1-5 times crossover freq)
%    delta_phi_rad     - Desired phase lead [radians]
%    delta_K           - Magnitude compensation factor [unitless]
%    wgc               - Crossover frequency [rad/s]
%
%  Outputs:
%    kp, ki, kd        - Proportional, integral, derivative gains
%    tl                - Time constant for filter pole [seconds]
%    type              - Controller type label ('PID')
%
function [kp,ki,kd,tl,type] = pid_controller_automatized(alfa,wgc_p,delta_phi_rad,delta_K,wgc)
        
        % Input validation
        if alfa <= 0
            error('pid_controller_automatized:InvalidAlfa', 'Parameter alfa must be positive');
        end
        if wgc_p <= 0
            error('pid_controller_automatized:InvalidWgcP', 'Parameter wgc_p must be positive');
        end
        if wgc <= 0
            error('pid_controller_automatized:InvalidWgc', 'Crossover frequency wgc must be positive');
        end
        
        type = 'PID';
        
        % Compute derivative and integral time constants
        Td = (tan(delta_phi_rad) + sqrt(tan(delta_phi_rad)^2 + 4/alfa)) / (2 * wgc); 
        Ti = alfa * Td; 
        
        % Compute PID gains
        kp = delta_K * cos(delta_phi_rad); 
        ki = kp / Ti;
        kd = kp * Td;
        
        % Compute filter time constant
        tl = 1 / (wgc_p * wgc); 

end