% foc_reference.m
% Reference simulation of a PMSM with field-oriented control (FOC), written in plain MATLAB.
% Purpose: an "answer key" used to cross-check the Simulink model.
% Motor parameters: Tripathi et al. (2023), Table 1. Gains: starting values, to be retuned.
%
% HOW TO RUN: put this file in your project folder, open MATLAB in that folder,
% type   foc_reference   in the Command Window and press Enter.

clear; clc; close all;

%% 1. Motor parameters
Rs  = 0.0186875;   % stator resistance (ohm)
Ld  = 6.5e-3;      % d-axis inductance (H)
Lq  = 6.5e-3;      % q-axis inductance (H)
lam = 0.175;       % permanent-magnet flux (Wb)
B   = 0.001;       % damping (N.m.s)
J   = 0.008;       % inertia (kg.m^2)
poles = 8;
p   = poles/2;     % pole pairs
Kt  = 1.5*p*lam;   % torque per amp when id = 0 (N.m/A)

%% 2. Controller gains (starting values)
alpha_c = 2000;               % current-loop bandwidth (rad/s)
Kp_c = alpha_c*Ld;            % current PI, proportional
Ki_c = alpha_c*Rs;            % current PI, integral
alpha_s = 200;                % speed-loop bandwidth (rad/s)
Kp_s = alpha_s*J/Kt;          % speed PI, proportional
Ki_s = Kp_s*alpha_s/5;        % speed PI, integral
iq_max = 20;                  % current limit (A)

%% 3. Simulation settings
dt = 1e-5;                    % time step (s)
T  = 0.8;                     % stop time (s)
N  = round(T/dt);
stride = 10;                  % log every 10th step

%% 4. Motor equations: x = [id; iq; wm]
f = @(x,Vd,Vq,TL) [ ...
    (Vd - Rs*x(1) + p*x(3)*Lq*x(2)) / Ld; ...
    (Vq - Rs*x(2) - p*x(3)*Ld*x(1) - p*x(3)*lam) / Lq; ...
    (1.5*p*(lam*x(2) + (Ld-Lq)*x(1)*x(2)) - TL - B*x(3)) / J ];

%% 5. Run the simulation
x = [0; 0; 0];
ei = 0; eid = 0; eiq = 0;                 % integrator states of the three PIs
nlog = floor((N-1)/stride) + 1;
L = zeros(nlog, 7);                       % t, wref, wm, id, iq, Te, TL
row = 0;

for k = 1:N
    t = (k-1)*dt;

    % test scenario
    if t < 0.2,     wref = 20;
    elseif t < 0.4, wref = 30;
    else            wref = 40;
    end
    if t < 0.5, TL = 5; else TL = 3; end

    % speed PI -> iq reference (with clamp and anti-windup)
    e = wref - x(3);
    u = Kp_s*e + Ki_s*ei;
    iq_ref = min(max(u, -iq_max), iq_max);
    if abs(u) < iq_max, ei = ei + e*dt; end

    % current PIs
    e_d = 0 - x(1);
    e_q = iq_ref - x(2);
    eid = eid + e_d*dt;
    eiq = eiq + e_q*dt;
    Vd_p = Kp_c*e_d + Ki_c*eid;
    Vq_p = Kp_c*e_q + Ki_c*eiq;

    % decoupling
    we = p*x(3);
    Vd = Vd_p - we*Lq*x(2);
    Vq = Vq_p + we*(Ld*x(1) + lam);

    % motor update (Runge-Kutta 4)
    k1 = f(x,           Vd, Vq, TL);
    k2 = f(x + dt/2*k1, Vd, Vq, TL);
    k3 = f(x + dt/2*k2, Vd, Vq, TL);
    k4 = f(x + dt*k3,   Vd, Vq, TL);
    x  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    if mod(k-1, stride) == 0
        row = row + 1;
        Te = 1.5*p*(lam*x(2) + (Ld-Lq)*x(1)*x(2));
        L(row,:) = [t, wref, x(3), x(1), x(2), Te, TL];
    end
end
L = L(1:row,:);
t = L(:,1); wref = L(:,2); w = L(:,3); id = L(:,4); iq = L(:,5); Te = L(:,6); TL = L(:,7);

%% 6. Metrics for the 20 -> 30 rad/s step at t = 0.2 s
sel = t >= 0.2 & t < 0.4;
ts_ = t(sel) - 0.2;  ws_ = w(sel);
y0 = 20; yf = 30;
overshoot = (max(ws_) - yf)/(yf - y0)*100;
i10 = find(ws_ >= y0 + 0.1*(yf-y0), 1);
i90 = find(ws_ >= y0 + 0.9*(yf-y0), 1);
rise_time = ts_(i90) - ts_(i10);
last_out = find(abs(ws_ - yf) > 0.02*yf, 1, 'last');
if isempty(last_out), settle_time = 0; else settle_time = ts_(last_out+1); end

% load step at t = 0.5 s (5 -> 3 N.m), speed reference 40 rad/s
sel2 = t >= 0.5 & t < 0.8;
w2 = w(sel2); t2 = t(sel2) - 0.5;
peak_after_load = max(w2);
last_out2 = find(abs(w2 - 40) > 0.02*40, 1, 'last');
if isempty(last_out2), recover = 0; else recover = t2(last_out2+1); end

fprintf('\n--- Metrics (your own Simulink model should give similar values) ---\n');
fprintf('Speed step 20->30 rad/s:  overshoot = %.2f %%, rise time = %.2f ms, settling (2%%) = %.2f ms\n', ...
        overshoot, rise_time*1e3, settle_time*1e3);
fprintf('Load step 5->3 N.m at 0.5 s: speed peaks at %.2f rad/s, back within 2%% after %.2f ms\n', ...
        peak_after_load, recover*1e3);
fprintf('Max |id| = %.4f A,  iq before load step = %.2f A,  after = %.2f A\n', ...
        max(abs(id)), iq(find(t>=0.49,1)), iq(find(t>=0.7,1)));

%% 7. Plots
if ~exist('results','dir'), mkdir('results'); end
fig = figure('Position',[100 100 900 800]);
subplot(3,1,1); plot(t, wref, 'r--', t, w, 'b'); grid on;
ylabel('Speed (rad/s)'); legend('Reference','Actual','Location','southeast');
title('FOC of a PMSM: speed tracking and load step');
subplot(3,1,2); plot(t, id, t, iq); grid on;
ylabel('Current (A)'); legend('id','iq');
subplot(3,1,3); plot(t, Te, 'b', t, TL, 'r--'); grid on;
ylabel('Torque (N.m)'); xlabel('Time (s)'); legend('Te','Load torque');
try
    exportgraphics(fig, 'results/fig_reference_run.png');
catch
    saveas(fig, 'results/fig_reference_run.png');
end
disp('Saved results/fig_reference_run.png');
