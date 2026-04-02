function [kp,ki,kd,tl,type] = pi_controller_lab_1(wgc_p,delta_phi_rad,delta_K,wgc)

        type = 'PI';
        kp = delta_K * cos(delta_phi_rad);
        ki = -wgc * delta_K * sin(delta_phi_rad);
        kd = 0;
        tl = 1 / (wgc_p * wgc); 
end