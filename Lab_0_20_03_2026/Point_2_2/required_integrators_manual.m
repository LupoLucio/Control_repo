function [number_of_integrators] = required_integrators_manual(P,signal_type)

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
        
        number_of_integrators = integrators - n_poli_origine;

end
