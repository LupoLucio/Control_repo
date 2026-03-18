function [kp,ki,kd,tl,type] = design_pid_controller(Mp,ts_5,P,alfa,wgc_p,signal_type)

% Compute the parameters

delta = log(1/Mp) / sqrt(pi^2 + (log(1/Mp))^2); 
wgc = 3 / (delta * ts_5); 
PM_des = 180/pi *(atan(2*delta / sqrt(sqrt(1+4*delta^4)-2*delta^2)));

[mag, phase] = bode(P, wgc);
mag = squeeze(mag); % Magnitude Compensation
phase = squeeze(phase);

delta_phi_deg = -180 + PM_des - phase;
delta_phi_rad = pi/180*delta_phi_deg; % Phase Lead
delta_K = 1/mag;

number_of_integrators = required_integrators(P,signal_type);


% Note: tl = 1 / (wgc_p * wgc); % 2 - 5 times wgc the parameter can be tuned


if (number_of_integrators == 0) 
    
    if (delta_phi_deg > 0 && delta_phi_deg < 90)

        [kp,ki,kd,tl,type] = pd_controller(wgc_p,delta_phi_rad,delta_K,wgc);
    
    elseif (delta_phi_deg > 0 && delta_phi_deg <180)

        [kp,ki,kd,tl,type] = pid_controller(alfa,wgc_p,delta_phi_rad,delta_K,wgc);
    
    else
         fprintf("Design with PID is not possible");
    end

elseif (number_of_integrators == 1) && (delta_phi_deg < 0 && delta_phi_deg > -90)

    if (delta_phi_deg < 0 && delta_phi_deg > -90)

        [kp,ki,kd,tl,type] = pi_controller(wgc_p,delta_phi_rad,delta_K,wgc);
        
    elseif (delta_phi_deg > 0 && delta_phi_deg <180)
    
        [kp,ki,kd,tl,type] = pid_controller(alfa,wgc_p,delta_phi_rad,delta_K,wgc);

    else

        fprintf("Design with PID is not possible");
    
    end

else 

    fprintf("Design with PID is not possible");
        
end
end

function [kp,ki,kd,tl,type] = pid_controller(alfa,wgc_p,delta_phi_rad,delta_K,wgc)
        type = 'PID';
        Td = (tan(delta_phi_rad) + sqrt(tan(delta_phi_rad)^2 + 4/alfa)) / (2 * wgc); 
        Ti = alfa * Td; 
        kp = delta_K * cos(delta_phi_rad); 
        ki = Kp / Ti;
        kd = Kp * Td;
        tl = 1 / (wgc_p * wgc); 

end
function [kp,ki,kd,tl,type] = pi_controller(wgc_p,delta_phi_rad,delta_K,wgc)

        type = 'PI';
        kp = delta_K * cos(delta_phi_rad);
        ki = -wgc * delta_K * sin(delta_phi_rad);
        kd = 0;
        tl = 1 / (wgc_p * wgc); 
end
function [kp,ki,kd,tl,type] = pd_controller(wgc_p,delta_phi_rad,delta_K,wgc)

        kp = delta_K * cos(delta_phi_rad);
        kd = (delta_K * sin(delta_phi_rad)) / wgc;
        ki = 0;
        type = 'PD';
        tl = 1 / (wgc_p * wgc); 
end

function [number_of_integrators] = required_integrators(P,signal_type)

soglia = 1e-6;

if signal_type == "impulse"
    integrators = 0;
elseif signal_type == "step"
    integrators = 1;
elseif signal_type == "linear ramp"
    integrators = 2;
elseif signal_type == "parabolic ramp"
    integrators = 3;
else
    fprintf("Error signal not known!");
end

tutti_i_poli = pole(P);

n_poli_origine = sum(abs(tutti_i_poli) < soglia);

number_of_integrators = n_poli_origine - integrators;

end