function [number_of_integrators] = required_integrators(numP,denP,signal_type)

soglia = 1e-6;

if signal == "impulse"
    integrators = 0;
elseif signal == "step"
    integrators = 1;
elseif signal == "linear ramp"
    integrators = 2;
elseif signal = "parabolic ramp"
    integrators = 3;
else
    fprintf("Error signal not known!");
end

tutti_i_poli = pole(P);

n_poli_origine = sum(abs(tutti_i_poli) < soglia);

number_of_integrators = n_poli_origine - integrators;

end