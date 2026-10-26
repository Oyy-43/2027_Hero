function p = optimize_pid(sim_fun, target, Ts, overshoot_weight, label)
    lb = [1.0,   0.0, 0.0,   2.0];
    ub = [15.0,  3.0, 1.5,  30.0];
    x0 = [8.0,   0.1, 0.2,   8.0];

    cost = @(p) pid_cost(p, sim_fun, target, Ts, overshoot_weight);
    opts = optimoptions('fmincon', 'Display','iter', ... 
        'MaxIterations', 100, 'Algorithm','sqp', 'StepTolerance', 1e-4);

    best_p = x0; best_J = inf;
    fprintf('\n>> 正在优化 [%s] 方案...\n', label);
    for trial = 1:3
        if trial == 1
            x_init = x0;
        else
            x_init = lb + (ub - lb) .* rand(1, 4);
        end
        try
            [p_i, J_i] = fmincon(cost, x_init, [], [], [], [], lb, ub, [], opts);
            if J_i < best_J
                best_J = J_i; best_p = p_i;
            end
        catch
        end
    end
    p = best_p;
    fprintf('  [%s] 完成 | Kp=%.2f, Ki=%.3f, Kd=%.3f, Ilim=%.2f\n', ...
        label, p(1), p(2), p(3), p(4));
end