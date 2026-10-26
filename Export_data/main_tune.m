%% ============================================================
%  main_tune.m (主脚本)
% ============================================================
clear; clc; close all;

%% ==================== 基础参数 ====================
Ts         = 0.001;         % 1000 Hz
MaxOut     = 30.0;          % PID 输出限幅（速度指令 rad/s）
OmegaSat   = 20.0;          % 速度环内部饱和（实际最大速度 rad/s）
Deadband   = 0.005;         % 死区 0.005 rad ≈ 0.29°
KdFilt     = 0.2;           % 微分滤波系数
OutFilt    = 1.0;           % 输出滤波系数
Tsim       = 4.0;           % 仿真时长
StepTarget = 1.57;          % 阶跃目标 1.57 rad

%% ==================== 1. 选择并读取数据 ====================
fprintf('【步骤 1/4】请在弹出的窗口中选择你的 CSV 文件...\n');
[file, path] = uigetfile('*.csv', '选择阶跃响应数据文件');
if isequal(file, 0)
    error('你取消了文件选择，程序退出。');
end
CSV_FILE = fullfile(path, file);
fprintf('已选择文件：%s\n', CSV_FILE);

T = readtable(CSV_FILE, 'VariableNamingRule','preserve');
t       = T.Time;
r_ang   = T.Angle_Target;
y_ang   = T.Angle_Measure;
r_spd   = T.Omega_Target;
y_spd   = T.Omega_Measure;

mask = isfinite(t) & isfinite(r_ang) & isfinite(y_ang) & isfinite(r_spd) & isfinite(y_spd);
t=t(mask); r_ang=r_ang(mask); y_ang=y_ang(mask); r_spd=r_spd(mask); y_spd=y_spd(mask);
[t, ia] = unique(t, 'stable');
r_ang=r_ang(ia); y_ang=y_ang(ia); r_spd=r_spd(ia); y_spd=y_spd(ia);

tq      = (t(1):Ts:t(end))';
r_ang_q = interp1(t, r_ang, tq, 'linear');
y_ang_q = interp1(t, y_ang, tq, 'linear');
r_spd_q = interp1(t, r_spd, tq, 'linear');
y_spd_q = interp1(t, y_spd, tq, 'linear');

dR = diff(r_ang_q);
idx = find(abs(dR) > 0.5) + 1;
iStep = idx(1);
tStep = tq(iStep);
fprintf('检测到第一个阶跃：t = %.4f s\n', tStep);

%% ==================== 2. 速度环辨识 ====================
fprintf('\n【步骤 2/4】正在辨识速度环模型...\n');
i0 = find(tq >= tStep - 0.05, 1);
i1 = find(tq >= tStep + 1.0, 1);
segIdx = i0:i1;

u_spd = r_spd_q(segIdx);
y_spd = y_spd_q(segIdx);
baseMask = tq(segIdx) < tStep;
u_spd = u_spd - mean(u_spd(baseMask));
y_spd = y_spd - mean(y_spd(baseMask));

data_spd = iddata(y_spd, u_spd, Ts);

opt = tfestOptions('Display','off');
Gv1 = tfest(data_spd, 1, 0, opt);
Gv2 = tfest(data_spd, 2, 0, opt);

fit1 = Gv1.Report.Fit.FitPercent;
fit2 = Gv2.Report.Fit.FitPercent;
fprintf('速度环辨识：1阶拟合 %.1f%%，2阶拟合 %.1f%%\n', fit1, fit2);

if fit2 > fit1 + 5
    Gv = Gv2;
else
    Gv = Gv1;
end

[num, den] = tfdata(Gv, 'v');
Gv = tf(num, den);
if Gv.Ts ~= 0
    Gv = d2c(Gv, 'tustin');
end
Gv = minreal(Gv);
Gv_d = c2d(Gv, Ts, 'tustin');
disp('速度环闭环模型 Gv(s)：'); disp(Gv);

%% ==================== 3. 位置环仿真函数 ====================
sim_fun = @(p) simulate_position_pid(p, StepTarget, Gv_d, Ts, ...
    MaxOut, OmegaSat, Deadband, KdFilt, OutFilt, Tsim);

%% ==================== 4. 多目标优化 ====================
fprintf('\n【步骤 3/4】开始优化（请稍等 10~30 秒）...\n');

% 确保工作路径包含以下文件
p_safe     = optimize_pid(sim_fun, StepTarget, Ts, 500, '保守');
p_balanced = optimize_pid(sim_fun, StepTarget, Ts, 100, '均衡');
p_fast     = optimize_pid(sim_fun, StepTarget, Ts,  20, '激进');

%% ==================== 5. 打印推荐结果 ====================
fprintf('\n【步骤 4/4】优化完成！以下是推荐参数：\n');
print_result('方案 1（保守，几乎无超调）',  p_safe,     sim_fun, StepTarget, Ts, MaxOut, Deadband);
print_result('方案 2（推荐，快 + 小超调）',  p_balanced, sim_fun, StepTarget, Ts, MaxOut, Deadband);
print_result('方案 3（激进，最快）',         p_fast,     sim_fun, StepTarget, Ts, MaxOut, Deadband);

%% ==================== 6. 绘图对比 ====================
[y_s, t] = sim_fun(p_safe);
[y_b, ~] = sim_fun(p_balanced);
[y_f, ~] = sim_fun(p_fast);

figure('Name','三组位置环参数阶跃响应对比');
plot(t, y_s/StepTarget, 'g-', 'LineWidth', 1.5); hold on;
plot(t, y_b/StepTarget, 'b-', 'LineWidth', 1.5);
plot(t, y_f/StepTarget, 'r-', 'LineWidth', 1.5);
yline(1, 'k--');
grid on; xlabel('时间 (s)'); ylabel('归一化角度');
legend('保守', '均衡', '激进', '目标', 'Location','southeast');
title('位置环闭环阶跃响应（含速度饱和 20 rad/s）');