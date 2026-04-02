function out = pid_metrics(P, Kp, Ki, Kd, Tl)

    s = tf('s');
    D_real_term = (Kd*s)/(1+Tl*s);
    
    I_term = Ki/s;
    
    P_term = Kp;
    
    C = P_term + I_term + D_real_term;
    
    T = feedback(C*P, 1);
    
    info = stepinfo(T);
    
    out.TF_closed_loop = T;
    out.settling_time = info.SettlingTime;
    out.rise_time = info.RiseTime;
    out.overshoot = info.Overshoot;
    out.final_value = dcgain(T);
    out.closed_loop_poles = pole(T);

    fprintf("\n=== PID ANALYSIS RESULTS ===\n");
    fprintf("Kp = %.4f, Ki = %.4f, Kd = %.4f, Tl = %.4f\n", Kp, Ki, Kd, Tl);
    fprintf("Rise time: %.4f s\n", out.rise_time);
    fprintf("Settling time: %.4f s\n", out.settling_time);
    fprintf("Overshoot: %.2f %%\n", out.overshoot);
    fprintf("Final value: %.4f\n", out.final_value);
    fprintf("Closed loop poles: \n");
    disp(out.closed_loop_poles);
    fprintf("====================================\n\n");
end