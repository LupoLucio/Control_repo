function out = pid_metrics(P, Kp, Ki, Kd, Tl)

    s = tf('s');
    
    % Termine derivativo con filtro
    D_real_term = (Kd*s)/(1 + Tl*s);
    
    % Termine integrativo
    I_term = Ki/s;
    
    % Termine proporzionale
    P_term = Kp;
    
    % Controllore PID
    C = P_term + I_term + D_real_term;
    
    % Funzione in anello chiuso
    T = feedback(C*P, 1);
    
    % Metriche con specifiche corrette:
    % - Settling time al 5%
    % - Rise time 5%–95%
    info = stepinfo(T, ...
        'SettlingTimeThreshold', 0.05);
    
    % Output
    out.ts_5 = info.SettlingTime;          % Settling time 5%
    out.overshoot = info.Overshoot;

    % Stampa risultati
    fprintf("\n=== PID ANALYSIS RESULTS ===\n");
    fprintf("Settling time (5%%): %.4f s\n", out.ts_5);
    fprintf("Overshoot: %.2f %%\n", out.overshoot);
    fprintf("====================================\n\n");
end