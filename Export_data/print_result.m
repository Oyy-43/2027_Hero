function print_result(label, p, sim_fun, target, Ts, MaxOut, Deadband)
    [y, t] = sim_fun(p);
    y_n = y / target;
    i10 = find(y_n >= 0.1, 1, 'first');
    i90 = find(y_n >= 0.9, 1, 'first');
    if isempty(i10) || isempty(i90), Tr = NaN;
    else, Tr = t(i90) - t(i10);
    end
    Overshoot = max(0, (max(y_n) - 1) * 100);
    idx_band  = find(abs(y_n - 1) > 0.02, 1, 'last');
    if isempty(idx_band), Ts2 = 0;
    else, Ts2 = t(min(idx_band + 1, length(t)));
    end
    Ess = abs(1 - y_n(end));

    fprintf('\n============================================================\n');
    fprintf('【%s】\n', label);
    fprintf('  Kp = %.3f   Ki = %.3f   Kd = %.3f   IntegralLimit = %.2f\n', ...
        p(1), p(2), p(3), p(4));
    fprintf('  上升时间(10-90%%) = %.3f s\n', Tr);
    fprintf('  超调量          = %.2f %%\n', Overshoot);
    fprintf('  调节时间(±2%%)   = %.3f s\n', Ts2);
    fprintf('  稳态误差        = %.5f rad\n', Ess);
    fprintf('\n  ---- 对应 alg_pid.c 初始化代码 ----\n');
    fprintf('  PID_Init(&pid_pos,\n');
    fprintf('      %.1ff,        // MaxOut\n', MaxOut);
    fprintf('      %.2ff,        // IntegralLimit\n', p(4));
    fprintf('      %.4ff,       // DeadBand\n', Deadband);
    fprintf('      0.001f,      // D_T\n');
    fprintf('      %.3ff, %.3ff, %.3ff, 0.0f,   // Kp, Ki, Kd, Kf\n', p(1), p(2), p(3));
    fprintf('      0.5f, 0.1f,  // 变积分 A, B\n');
    fprintf('      1.0f,        // 输出滤波\n');
    fprintf('      0.2f,        // 微分滤波\n');
    fprintf('      Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | DerivativeFilter\n');
    fprintf('  );\n');
    fprintf('============================================================\n');
end