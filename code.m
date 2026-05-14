%% Active Suspension Control System - v2 (Fixed & Improved)
% Plant: G(s) = 1 / (s^2 + 3s + 2)
% Controllers: PD and PID
% Goals: Settling time < 5s, minimize oscillations, improve damping

clear; clc; close all;

%% =========================================================
%  1. SYSTEM MODEL
%% =========================================================
G = tf([1], [1 3 2]);

fprintf('=== System Model ===\n');
disp(G);

poles_ol = pole(G);
[wn_ol, zeta_ol] = damp(G);
fprintf('Open-Loop Poles:              %.4f,  %.4f\n', poles_ol(1), poles_ol(2));
fprintf('Natural Frequencies (rad/s):  %.4f,  %.4f\n', wn_ol(1), wn_ol(2));
fprintf('Damping Ratios:               %.4f,  %.4f\n\n', zeta_ol(1), zeta_ol(2));

%% =========================================================
%  2. OPEN-LOOP STEP RESPONSE
%% =========================================================
t = 0:0.01:15;
[y_ol, t_ol] = step(G, t);
info_ol = stepinfo(G);

fprintf('=== Open-Loop Performance ===\n');
fprintf('Rise Time:     %.4f s\n', info_ol.RiseTime);
fprintf('Settling Time: %.4f s\n', info_ol.SettlingTime);
fprintf('Overshoot:     %.4f %%\n', info_ol.Overshoot);
fprintf('SS Value:      %.4f  (expected = 1/2 = 0.5 by DC gain)\n\n', dcgain(G));

%% =========================================================
%  3. PD CONTROLLER
%  C_PD(s) = Kp + Kd*s
%  Note: PD has NO integral => steady-state error exists
%  SS error = 1/(1 + Kp*dc_gain)
%% =========================================================
Kp_pd = 20;
Kd_pd = 8;
C_PD  = tf([Kd_pd Kp_pd], [1]);
G_cl_PD = feedback(C_PD * G, 1);

[y_pd, t_pd] = step(G_cl_PD, t);
info_pd = stepinfo(G_cl_PD);
ss_pd   = dcgain(G_cl_PD);   % will be < 1 due to no integral

fprintf('=== PD Controller (Kp=%.0f, Kd=%.0f) ===\n', Kp_pd, Kd_pd);
fprintf('Closed-Loop Poles: '); fprintf('%.4f  ', pole(G_cl_PD)); fprintf('\n');
fprintf('Rise Time:         %.4f s\n', info_pd.RiseTime);
fprintf('Settling Time:     %.4f s\n', info_pd.SettlingTime);
fprintf('Overshoot:         %.4f %%\n', info_pd.Overshoot);
fprintf('Steady-State Val:  %.4f  (SS error = %.4f)\n\n', ss_pd, 1-ss_pd);

%% =========================================================
%  4. PID CONTROLLER
%  C_PID(s) = Kp + Ki/s + Kd*s
%  Integral action => zero SS error, SS value = 1.0
%% =========================================================
Kp_pid = 30;
Ki_pid = 20;
Kd_pid = 10;
C_PID   = tf([Kd_pid Kp_pid Ki_pid], [1 0]);
G_cl_PID = feedback(C_PID * G, 1);

[y_pid, t_pid] = step(G_cl_PID, t);
info_pid = stepinfo(G_cl_PID);
ss_pid   = dcgain(G_cl_PID);   % should be = 1.0

fprintf('=== PID Controller (Kp=%.0f, Ki=%.0f, Kd=%.0f) ===\n', Kp_pid, Ki_pid, Kd_pid);
fprintf('Closed-Loop Poles: '); fprintf('%.4f  ', pole(G_cl_PID)); fprintf('\n');
fprintf('Rise Time:         %.4f s\n', info_pid.RiseTime);
fprintf('Settling Time:     %.4f s\n', info_pid.SettlingTime);
fprintf('Overshoot:         %.4f %%\n', info_pid.Overshoot);
fprintf('Steady-State Val:  %.4f  (SS error = %.6f)\n\n', ss_pid, 1-ss_pid);

%% =========================================================
%  5. DAMPING ANALYSIS
%% =========================================================
[wn_pd,  zeta_pd]  = damp(G_cl_PD);
[wn_pid, zeta_pid] = damp(G_cl_PID);

fprintf('=== Damping Analysis ===\n');
fprintf('Open-Loop Damping Ratios:       '); fprintf('%.4f  ', zeta_ol);  fprintf('\n');
fprintf('PD  Closed-Loop Damping Ratios: '); fprintf('%.4f  ', zeta_pd);  fprintf('\n');
fprintf('PID Closed-Loop Damping Ratios: '); fprintf('%.4f  ', zeta_pid); fprintf('\n\n');

%% =========================================================
%  6. PLOTS
%% =========================================================

% ---- Figure 1: Step Response ----
figure('Name','Step Response Comparison','Color','k','Position',[50 50 950 580]);
set(gca,'Color','k');
hold on;
p1 = plot(t_ol,  y_ol,  'b-',  'LineWidth', 2.2, 'DisplayName', 'Uncontrolled (Open-Loop)');
p2 = plot(t_pd,  y_pd,  'r--', 'LineWidth', 2.2, 'DisplayName', sprintf('PD  (Ts=%.2fs, SSval=%.2f)', info_pd.SettlingTime, ss_pd));
p3 = plot(t_pid, y_pid, 'g-',  'LineWidth', 2.2, 'DisplayName', sprintf('PID (Ts=%.2fs, SSval=1.00)', info_pid.SettlingTime));
yline(1,   'w:',  'Reference = 1', 'LineWidth', 1.2, 'LabelHorizontalAlignment','right');
yline(0.5, 'c:',  'OL SS = 0.5',   'LineWidth', 1.0, 'LabelHorizontalAlignment','right');
xline(5,   'm-.', 'Ts limit = 5s', 'LineWidth', 1.4, 'LabelHorizontalAlignment','right','Color','m');
hold off;
set(gca,'Color',[0.1 0.1 0.1],'XColor','w','YColor','w','GridColor','w','GridAlpha',0.2);
grid on;
xlabel('Time (s)',         'Color','w', 'FontSize', 13);
ylabel('Body Displacement','Color','w', 'FontSize', 13);
title('Active Suspension: Step Response Comparison', 'Color','w', 'FontSize', 14, 'FontWeight','bold');
legend([p1 p2 p3], 'Location','best','FontSize',10,'TextColor','w','Color',[0.15 0.15 0.15]);
xlim([0 15]);

% ---- Figure 2: Pole-Zero Map ----
figure('Name','Pole-Zero Map','Color','k','Position',[50 50 750 520]);
hold on;
plot(real(poles_ol),           imag(poles_ol),           'bx', 'MarkerSize',16,'LineWidth',3,   'DisplayName','Open-Loop Poles');
plot(real(pole(G_cl_PD)),      imag(pole(G_cl_PD)),      'ro', 'MarkerSize',12,'LineWidth',2.5, 'DisplayName','PD Closed-Loop');
plot(real(pole(G_cl_PID)),     imag(pole(G_cl_PID)),     'gs', 'MarkerSize',12,'LineWidth',2.5, 'DisplayName','PID Closed-Loop');
hold off;
set(gca,'Color',[0.1 0.1 0.1],'XColor','w','YColor','w','GridColor','w','GridAlpha',0.2);
grid on; xline(0,'w--','LineWidth',0.8); yline(0,'w--','LineWidth',0.8);
xlabel('Real Axis',      'Color','w','FontSize',13);
ylabel('Imaginary Axis', 'Color','w','FontSize',13);
title('Pole-Zero Map: Open-Loop vs Controlled','Color','w','FontSize',14,'FontWeight','bold');
legend('Location','best','FontSize',10,'TextColor','w','Color',[0.15 0.15 0.15]);

% ---- Figure 3: Bode Plot ----
figure('Name','Bode Plot','Color','k','Position',[50 50 850 580]);
bode(G, C_PD*G, C_PID*G);
legend({'Plant G(s)','PD Open-Loop L(s)','PID Open-Loop L(s)'},'FontSize',10);
title('Bode Plot: Plant vs Controlled Open-Loop','FontSize',14,'FontWeight','bold');
grid on;

% ---- Figure 4: Performance Bar Charts ----
figure('Name','Performance Metrics','Color','k','Position',[50 50 900 450]);

metrics  = [info_ol.SettlingTime, info_pd.SettlingTime,  info_pid.SettlingTime;
            info_ol.Overshoot,    info_pd.Overshoot,     info_pid.Overshoot;
            dcgain(G),            ss_pd,                 ss_pid];
labels   = {'Open-Loop','PD','PID'};

subplot(1,3,1);
b = bar(metrics(1,:), 0.5);
b.FaceColor = 'flat';
b.CData = [0.2 0.4 0.9; 0.9 0.3 0.2; 0.2 0.8 0.4];
set(gca,'XTickLabel',labels,'FontSize',10,'Color',[0.1 0.1 0.1],'XColor','w','YColor','w');
yline(5,'r--','LineWidth',2); ylabel('Settling Time (s)','Color','w');
title('Settling Time','Color','w','FontWeight','bold'); grid on;

subplot(1,3,2);
b2 = bar(metrics(2,:), 0.5);
b2.FaceColor = 'flat';
b2.CData = [0.2 0.4 0.9; 0.9 0.3 0.2; 0.2 0.8 0.4];
set(gca,'XTickLabel',labels,'FontSize',10,'Color',[0.1 0.1 0.1],'XColor','w','YColor','w');
ylabel('Overshoot (%)','Color','w');
title('Overshoot','Color','w','FontWeight','bold'); grid on;

subplot(1,3,3);
b3 = bar(metrics(3,:), 0.5);
b3.FaceColor = 'flat';
b3.CData = [0.2 0.4 0.9; 0.9 0.3 0.2; 0.2 0.8 0.4];
set(gca,'XTickLabel',labels,'FontSize',10,'Color',[0.1 0.1 0.1],'XColor','w','YColor','w');
yline(1,'r--','LineWidth',1.5,'Label','Target=1.0');
ylabel('Steady-State Value','Color','w');
title('Steady-State Value','Color','w','FontWeight','bold'); grid on;

%% =========================================================
%  7. SUMMARY TABLE
%% =========================================================
fprintf('====================================================================\n');
fprintf('                     PERFORMANCE SUMMARY\n');
fprintf('====================================================================\n');
fprintf('%-25s %-12s %-12s %-12s\n','Metric','Open-Loop','PD','PID');
fprintf('--------------------------------------------------------------------\n');
fprintf('%-25s %-12.4f %-12.4f %-12.4f\n','Rise Time (s)',       info_ol.RiseTime,     info_pd.RiseTime,     info_pid.RiseTime);
fprintf('%-25s %-12.4f %-12.4f %-12.4f\n','Settling Time (s)',   info_ol.SettlingTime, info_pd.SettlingTime, info_pid.SettlingTime);
fprintf('%-25s %-12.4f %-12.4f %-12.4f\n','Overshoot (%%)',      info_ol.Overshoot,    info_pd.Overshoot,    info_pid.Overshoot);
fprintf('%-25s %-12.4f %-12.4f %-12.4f\n','Steady-State Value',  dcgain(G),            ss_pd,                ss_pid);
fprintf('%-25s %-12.4f %-12.4f %-12.4f\n','SS Error',            1-dcgain(G),          1-ss_pd,              1-ss_pid);
fprintf('====================================================================\n');

fprintf('\nSettling time < 5s requirement:\n');
fprintf('  PD  Controller: %s\n', check(info_pd.SettlingTime  < 5));
fprintf('  PID Controller: %s\n', check(info_pid.SettlingTime < 5));

fprintf('\nZero Steady-State Error:\n');
fprintf('  PD  Controller: %s  (SS error = %.4f — expected for PD)\n', check(abs(1-ss_pd)<0.01),  1-ss_pd);
fprintf('  PID Controller: %s  (SS error = %.6f)\n\n',                  check(abs(1-ss_pid)<0.01), 1-ss_pid);

%% Helper
function s = check(cond)
    if cond; s = 'PASS  checkmark'; else; s = 'FAIL  X'; end
end
