function [kp,ki,kd,tl,type] = pd_controller_manual(wgc_p,delta_phi_rad,delta_K,wgc)

        kp = delta_K * cos(delta_phi_rad);
        kd = (delta_K * sin(delta_phi_rad)) / wgc;
        ki = 0;
        type = 'PD';
        tl = 1 / (wgc_p * wgc); 
end