function [y_angle, t_out] = simulate_position_pid(p, target, Gv_d, Ts, ...
    MaxOut, OmegaSat, deadband, kd_filt, out_filt, Tsim)
    Kp = p(1); Ki = p(2); Kd = p(3); Ilimit = p(4);
    sys_d_ss = ss(Gv_d);
    [A, B, C, D] = ssdata(sys_d_ss);
    n_x = size(A, 1);
    x_omega = zeros(n_x, 1);
    N = round(Tsim / Ts);
    y_angle = zeros(N+1, 1);
    t_out   = (0:N)' * Ts;
    Iout = 0; Dout_filt = 0; Out_last = 0; Last_Err = 0; Last_Meas = 0;

    for k = 1:N
        angle = y_angle(k);
        Err   = target - angle;
        if abs(Err) > deadband
            Pout = Kp * Err;
            ITerm = Ki * (Err + Last_Err) / 2 * Ts;
            Iout_new = Iout + ITerm;
            Iout_new = max(min(Iout_new, Ilimit), -Ilimit); 
            Dout_raw  = Kd * (Last_Meas - angle) / Ts;
            Dout_filt = Dout_raw * kd_filt + Dout_filt * (1 - kd_filt);
            u = Pout + Iout_new + Dout_filt;
            if out_filt < 1
                u = u * out_filt + Out_last * (1 - out_filt);
            end
            u = max(min(u, MaxOut), -MaxOut);
            Iout = Iout_new;
        else
            u = 0;
        end
        u_sat = max(min(u, OmegaSat), -OmegaSat);
        omega   = C * x_omega + D * u_sat;
        x_omega = A * x_omega + B * u_sat;
        y_angle(k+1) = angle + omega * Ts;
        Last_Err  = Err;
        Last_Meas = angle;
        Out_last  = u;
    end
end