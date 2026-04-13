%% DESIGN_PID_CONTROLLER_AUTOMATIZED Automatic PID controller design using frequency domain
%  Designs a PID/PD/PI controller automatically based on desired closed-loop
%  performance metrics (overshoot and settling time) and plant characteristics.
%
%  Syntax: [Kp,Ki,Kd,tl,type] = design_pid_controller_automatized(Mp,ts_5,P,alfa,wgc_p,signal_type)
%
%  Inputs:
%    Mp              - Desired overshoot [unitless, 0-1, e.g., 0.05 for 5%]
%    ts_5            - Desired 5% settling time [seconds]
%    P               - Plant transfer function [tf object]
%    alfa            - PID tuning ratio [unitless, typically 4-10]
%    wgc_p           - Frequency weighting factor [unitless, typically 1-5]
%    signal_type     - Reference signal type: 'impulse', 'step', 'linear ramp', 'parabolic ramp'
%
%  Outputs:
%    Kp, Ki, Kd      - PID controller gains
%    tl              - Filter time constant [seconds]
%    type            - Controller type: 'PD', 'PI', or 'PID'
%
%  Note:
%    The function selects among PD, PI, or PID structures automatically
%    based on plant characteristics and signal type.
%
function [Kp,Ki,Kd,tl,type] = design_pid_controller_automatized(Mp,ts_5,P,alfa,wgc_p,signal_type)

        % Input validation
        if ~(0 < Mp && Mp < 1)
            error('design_pid_controller_automatized:InvalidMp', ...
                  sprintf('Overshoot Mp must be in (0, 1), got %.3f', Mp));
        end
        if ts_5 <= 0
            error('design_pid_controller_automatized:InvalidTs5', ...
                  'Settling time ts_5 must be positive');
        end
        if ~isa(P, 'tf') && ~isa(P, 'ss') && ~isa(P, 'zpk')
            error('design_pid_controller_automatized:InvalidPlant', ...
                  'Plant P must be a transfer function (tf, ss, or zpk object)');
        end
        if alfa <= 0
            error('design_pid_controller_automatized:InvalidAlfa', ...
                  'Parameter alfa must be positive');
        end
        if wgc_p <= 0
            error('design_pid_controller_automatized:InvalidWgcP', ...
                  'Parameter wgc_p must be positive');
        end
        
        % Compute desired frequency response characteristics
        delta = log(1/Mp) / (sqrt(pi^2 + (log(1/Mp))^2)); 
        wgc = 3 / (delta * ts_5); 
        PM_des = 180/pi *(atan(2*delta / sqrt(sqrt(1+4*delta^4)-2*delta^2)));
        
        % Evaluate plant at crossover frequency
        [mag, phase] = bode(P, wgc);
        mag = squeeze(mag);
        phase = squeeze(phase);
        
        % Compute required phase lead and magnitude compensation
        delta_phi_deg = PM_des - (180 + phase);
        delta_phi_rad = pi/180*delta_phi_deg;
        delta_K = 1/mag;
        
        % Determine required controller structure based on plant type
        number_of_integrators = required_integrators_automatized(P,signal_type);
        
        
        % Select controller structure based on phase margin requirement and plant integrators
        design_succeeded = false;
        
        if number_of_integrators == 0 
            % Plant has no integrators - need PD or PID
            if delta_phi_deg > 0 && delta_phi_deg < 90
                % Small lead requirement - use PD
                [Kp,Ki,Kd,tl,type] = pd_controller_automatized(wgc_p,delta_phi_rad,delta_K,wgc);
                design_succeeded = true;
                if isnan(Kp) || Kp <= 0
                    warning('design_pid_controller_automatized:NegativeKp', ...
                            'PD gain Kp is negative (%.6f). Check phase margin specification.', Kp);
                end
                fprintf('[PID Design] delta_phi: %.1f°, Strategy: PD\n', delta_phi_deg);
            
            elseif delta_phi_deg > 0 && delta_phi_deg < 180
                % Large lead requirement - use PID
                [Kp,Ki,Kd,tl,type] = pid_controller_automatized(alfa,wgc_p,delta_phi_rad,delta_K,wgc);
                design_succeeded = true;
                fprintf('[PID Design] delta_phi: %.1f°, Strategy: PID\n', delta_phi_deg);
            end
        
        elseif number_of_integrators == 1
            % Plant has 1 integrator - need PI or additional pole from PID
            if delta_phi_deg < 0 && delta_phi_deg > -90
                % Small lag acceptable - use PI
                [Kp,Ki,Kd,tl,type] = pi_controller_automatized(wgc_p,delta_phi_rad,delta_K,wgc);
                design_succeeded = true;
                fprintf('[PID Design] delta_phi: %.1f°, Strategy: PI\n', delta_phi_deg);
                
            elseif delta_phi_deg > 0 && delta_phi_deg < 180
                % Need lead compensation with existing integrator - use PID
                [Kp,Ki,Kd,tl,type] = pid_controller_automatized(alfa,wgc_p,delta_phi_rad,delta_K,wgc);
                design_succeeded = true;
                fprintf('[PID Design] delta_phi: %.1f°, Strategy: PID\n', delta_phi_deg);
            end
        end
        
        % Error handling for unsuccessful design
        if ~design_succeeded
            error('design_pid_controller_automatized:DesignFailed', ...
                  sprintf('PID design failed: n_integrators=%d, delta_phi=%.1f° (required > 0). \nTry increasing overshoot Mp or settling time ts_5.', ...
                  number_of_integrators, delta_phi_deg));
        end

end









