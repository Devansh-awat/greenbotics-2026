function R = power_budget()
% power_budget  Electrical power budget for the Greenbotics WRO vehicle.
%
%   Answers three questions the team must get right BEFORE the competition:
%     1. What peak current must the battery deliver? (C-rate check)
%     2. What peak current must the 5 V regulator deliver? (brownout check
%        — a Raspberry Pi 5 resets below 4.75 V; this kills a round)
%     3. How long does one battery charge last? (rounds per charge)
%
%   All component numbers marked [MEASURE] are datasheet/typical values —
%   replace them with multimeter measurements of the actual vehicle for
%   the engineering journal.
%
%   Prints a scenario table, saves results/power_budget.png, and returns
%   the numbers in struct R.

% Values marked [MEASURED <date>] were taken on the actual vehicle by the
% team; remaining [MEASURE] items still use estimates.

% ── Battery (3S LiPo) ──────────────────────────────────────────────────────
bat.V_nom    = 11.1;     % V   [MEASURED 2026-07] 3S pack
bat.cap_Ah   = 1.5;      % Ah  [MEASURED 2026-07] 1500 mAh label
bat.C_rate   = 25;       % [MEASURE] pending final pack choice (UN38.3 for travel)
bat.R_int    = 0.045;    % Ohm [MEASURE] (V_unloaded - V_loaded)/I

% ── 5 V rail: XY-3606 buck module (Pi + servo + sensors) ──────────────────
reg.V_out    = 5.0;      % V   XY-3606 (holds 5.2 V at 6 A per bench test)
reg.eff      = 0.90;     % buck efficiency
reg.I_rated  = 5.0;      % A   [MEASURED 2026-07] XY-3606 datasheet: 5 A

% ── Loads ──────────────────────────────────────────────────────────────────
% Raspberry Pi + camera (5 V rail) — PMIC readings over 15 s windows
pi5.P_idle   = 1.93;     % W  [MEASURED 2026-07] idle baseline
pi5.P_cv     = 5.71;     % W  [MEASURED 2026-07] full vision processing
pi5.P_peak   = 7.0;      % W  measured avg + margin for wifi/log bursts

% Steering servo (5 V rail)
servo.I_idle  = 0.05;    % A  holding position
servo.I_move  = 0.20;    % A  [MEASURE] normal steering activity
servo.I_stall = 0.80;    % A  [MEASURED 2026-07] approx at stall

% Drive motor via TB6612FNG (battery rail)
motor.I_cruise = 0.15;   % A  derived from measured battery totals (see check)
motor.I_avoid  = 0.50;   % A  [MEASURE] accel/decel during maneuvers
motor.I_stall  = 2.80;   % A  [MEASURE] wheel blocked (TB6612 peak limit 3.2 A)

% Small sensors: BNO055 + 2x VL53 ToF + status LED (5 V/3V3 rails)
sens.P = 0.6;            % W  combined

% Whole-robot cross-check (multimeter in series with the battery):
meas.Ibat_idle   = 0.40; % A  [MEASURED 2026-07] robot idle
meas.Ibat_active = 0.80; % A  [MEASURED 2026-07] driving, motor not stalled

% ── Scenarios ──────────────────────────────────────────────────────────────
% Each row: {name, Pi power, servo current, motor current}
scen = { 'Idle (setup)',      pi5.P_idle, servo.I_idle,  0.0
         'Cruise (lap)',      pi5.P_cv,   servo.I_move,  motor.I_cruise
         'Avoid maneuver',    pi5.P_peak, servo.I_move,  motor.I_avoid
         'Worst case (stall)',pi5.P_peak, servo.I_stall, motor.I_stall };

n = size(scen,1);
I5    = zeros(n,1);   % 5 V rail current
Ibat  = zeros(n,1);   % battery current
Vsag  = zeros(n,1);   % battery terminal voltage under load
Pbat  = zeros(n,1);
for i = 1:n
    P5      = scen{i,2} + sens.P + reg.V_out*scen{i,3};   % W on 5 V rail
    I5(i)   = P5 / reg.V_out;
    P_batt  = P5/reg.eff + bat.V_nom*scen{i,4};           % motor on Vbat
    Ibat(i) = P_batt / bat.V_nom;
    Vsag(i) = bat.V_nom - Ibat(i)*bat.R_int;
    Pbat(i) = P_batt;
end

% ── Report ─────────────────────────────────────────────────────────────────
fprintf('\n================ POWER BUDGET — Greenbotics WRO vehicle ================\n');
fprintf('%-20s %10s %12s %12s %10s\n', 'Scenario','5V rail A','Battery A','Battery V','Power W');
for i = 1:n
    fprintf('%-20s %9.2f %12.2f %12.2f %10.1f\n', scen{i,1}, I5(i), Ibat(i), Vsag(i), Pbat(i));
end

I5_peak   = max(I5);
Ibat_peak = max(Ibat);
runtime_h = bat.cap_Ah / Ibat(2);                 % at cruise
rounds    = floor(runtime_h*3600 / 240);          % 3-min round + margin

fprintf('\nKey checks:\n');
fprintf('  Battery C-rate needed : %.1f C  (pack rated %d C)  -> %s\n', ...
        Ibat_peak/bat.cap_Ah, bat.C_rate, pass(Ibat_peak/bat.cap_Ah < bat.C_rate));
fprintf('  5V regulator needed   : %.1f A peak (rated %.1f A, margin %.1fx) -> %s\n', ...
        I5_peak, reg.I_rated, reg.I_rated/I5_peak, pass(I5_peak < reg.I_rated));
if I5_peak >= reg.I_rated
    fprintf('  *** FINDING: a %.0f A regulator BROWNS OUT the Pi when the\n', reg.I_rated);
    fprintf('      servo stalls (%.1f A needed). Use a >= %.0f A UBEC or move the\n', ...
            I5_peak, ceil(I5_peak)+1);
    fprintf('      servo to its own regulator. A Pi reset mid-round = 0 points.\n');
end
fprintf('  Runtime at cruise     : %.0f min  (~%d full rounds per charge)\n', ...
        runtime_h*60, rounds);
fprintf('  Worst-case V sag      : %.2f V (cutoff 9.0 V) -> %s\n', ...
        min(Vsag), pass(min(Vsag) > 9.0));
fprintf('\nModel vs whole-robot measurement (battery current):\n');
fprintf('  Idle   : model %.2f A  vs measured %.2f A\n', Ibat(1), meas.Ibat_idle);
fprintf('  Cruise : model %.2f A  vs measured %.2f A\n', Ibat(2), meas.Ibat_active);
fprintf('=========================================================================\n');

% ── Figure ─────────────────────────────────────────────────────────────────
here = fileparts(mfilename('fullpath'));
res  = fullfile(here,'results');
if ~exist(res,'dir'), mkdir(res); end

fig = figure('Visible','off','Color','w','Position',[60 60 1350 460]);
try, fig.Theme = 'light'; catch, end

subplot(1,3,1);
bar(categorical(scen(:,1), scen(:,1)), Ibat, 'FaceColor',[0.25 0.5 0.85]);
ylabel('Battery current (A)'); grid on;
yline(bat.cap_Ah*bat.C_rate, 'r--', sprintf('pack limit %.0f A', bat.cap_Ah*bat.C_rate));
title('Battery current by scenario');

subplot(1,3,2);
bar(categorical(scen(:,1), scen(:,1)), I5, 'FaceColor',[0.9 0.5 0.15]); hold on;
yline(reg.I_rated, 'r--', sprintf('regulator rating %.0f A', reg.I_rated));
ylabel('5 V rail current (A)'); grid on;
title('5 V rail vs regulator rating');

subplot(1,3,3);
caps = 1.0:0.2:4.0;
plot(caps, caps/Ibat(2)*60, 'LineWidth',2); hold on;
plot(bat.cap_Ah, runtime_h*60, 'ro', 'MarkerSize',10, 'LineWidth',2);
text(bat.cap_Ah+0.05, runtime_h*60, sprintf('  %.0f min @ %.1f Ah', runtime_h*60, bat.cap_Ah));
xlabel('battery capacity (Ah)'); ylabel('cruise runtime (min)'); grid on;
title('Runtime vs battery capacity');

png = fullfile(res, 'power_budget.png');
saveas(fig, png); close(fig);
fprintf('Figure saved: %s\n', png);

R = struct('I5_peak',I5_peak, 'Ibat_peak',Ibat_peak, ...
           'runtime_min',runtime_h*60, 'Vsag_min',min(Vsag), ...
           'I5',I5, 'Ibat',Ibat, 'Vsag',Vsag);
end

function s = pass(ok)
    if ok, s = 'OK'; else, s = '*** FAIL ***'; end
end
