function J = pid_cost(p, sim_fun, target, Ts, overshoot_weight)
    if any(~isfinite(p)) || p(1) <= 0 || p(2) < 0 || p(3) < 0 || p(4) <= 0
        J = 1e8; return;
    end
    try
        [y, t] = sim_fun(p);
    catch
        J = 1e8; return;
    end
    if any(~isfinite(y)) || max(y) > 5 * target
        J = 1e8; return;
    end
    y_n = y / target;
    ITAE = sum(t .* abs(1 - y_n)) * Ts;
    overshoot = max(0, max(y_n) - 1);
    ess = abs(1 - y_n(end));
    J = ITAE + overshoot_weight * overshoot^2 + 1e6 * ess^2;
end