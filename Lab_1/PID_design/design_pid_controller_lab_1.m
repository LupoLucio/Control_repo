function [Kp,Ki,Kd,tl,type] = design_pid_controller_lab_1(Mp,ts_5,P,alfa,wgc_p,signal_type)

        % Compute the parameters
        
        delta = log(1/Mp) / (sqrt(pi^2 + (log(1/Mp))^2)); 
        wgc = 3 / (delta * ts_5); 
        PM_des = 180/pi *(atan(2*delta / sqrt(sqrt(1+4*delta^4)-2*delta^2)));
        
        [mag, phase] = bode(P, wgc);
        mag = squeeze(mag); % Magnitude Compensation
        phase = squeeze(phase);
        
        delta_phi_deg = PM_des - (180 + phase);
        delta_phi_rad = pi/180*delta_phi_deg; % Phase Lead
        delta_K = 1/mag;
        
        number_of_integrators = required_integrators_lab_1(P,signal_type);
        
        
        % Note: tl = 1 / (wgc_p * wgc); % 2 - 5 times wgc the parameter can be tuned
        
        
        if (number_of_integrators == 0) 
            
            if (delta_phi_deg > 0 && delta_phi_deg < 90)
        
                [Kp,Ki,Kd,tl,type] = pd_controller_lab_1(wgc_p,delta_phi_rad,delta_K,wgc);

            
            elseif (delta_phi_deg > 0 && delta_phi_deg <180)
        
                [Kp,Ki,Kd,tl,type] = pid_controller_lab_1(alfa,wgc_p,delta_phi_rad,delta_K,wgc);

            else 
                Kp = 0;
                Ki = 0;
                Kd = 0;
                tl = 0;
                type = 0;
            end

        
        elseif (number_of_integrators == 1)
        
            if (delta_phi_deg < 0 && delta_phi_deg > -90)
        
                [Kp,Ki,Kd,tl,type] = pi_controller_lab_1(wgc_p,delta_phi_rad,delta_K,wgc);
                
            elseif (delta_phi_deg > 0 && delta_phi_deg <180)
            
                [Kp,Ki,Kd,tl,type] = pid_controller_lab_1(alfa,wgc_p,delta_phi_rad,delta_K,wgc);
            else 
                 Kp = 0;
                 Ki = 0;
                 Kd = 0;
                 tl = 0;
                 type = 0;
            end
        else 
                 Kp = 0;
                 Ki = 0;
                 Kd = 0;
                 tl = 0;
                 type = 0;
        end
        









