function [kp,ki,kd,tl,type] = pid_controller_automatized(alfa,wgc_p,delta_phi_rad,delta_K,wgc)
        type = 'PID';
        Td = (tan(delta_phi_rad) + sqrt(tan(delta_phi_rad)^2 + 4/alfa)) / (2 * wgc); 
        Ti = alfa * Td; 
        kp = delta_K * cos(delta_phi_rad); 
        ki = kp / Ti;
        kd = kp * Td;
        tl = 1 / (wgc_p * wgc); 

end