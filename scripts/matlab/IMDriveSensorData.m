%% Parameters for Three-Phase Asynchronous Drive with Sensor Control 
% Nameplate-based bases (Delta, 220 V)
%   P_mech ≈ 180 W
%   V_LL   = 220 V
%   I_N    = 1.09 A
%   f_N    = 50 Hz
%   n_r    ≈ 1350 rpm
%   cos(phi) ≈ 0.68
%   p      = 2 pole pairs (4 poles)


%% Induction machine parameters
Pn  = 180;       % W,  Mechanical Power
Sn  = 415;       % VA, base power ~ S_base = sqrt(3)*220*1.09 ≈ 415 VA
Vn  = 220;       % V,  rms phase-to-phase, rated voltage
fn  = 50;        % Hz, rated frequency
In  = 1.09;      % A,  Nominal RMS current (delta connection)

% Per-unit machine parameters (approximate, referred to stator, per phase)
% These come from an estimated equivalent circuit for a small 0.18 kW motor.
Rs  = 0.150;    % pu, stator resistance
Lls = 0.11;     % pu, stator leakage inductance
Rr  = 0.13;     % pu, rotor resistance, referred to the stator side
Llr = 0.11;     % pu, rotor leakage inductance, referred to the stator side
Lm  = 1.360;    % pu, magnetizing inductance

% Derived total inductances in pu
Lr = Llr+Lm;    % pu, rotor inductance
Ls = Lls+Lm;    % pu, stator inductance

% Mechanical parameters (rough guess)
J = 0.0004;  % kg*m^2 
H = 0.0119;  % s, moment of inertia
F = 0.0106;  % pu,friction coefficient
p = 2;       % pole pairs (4-pole machine)

% Base quantities (machine side)
Iph = In / sqrt(3);          % A, Phase RMS current
Vbase = Vn * sqrt(2);        % V, base voltage, peak
Ibase = Iph * sqrt(2);       % A, base current, peak
Zbase = Vbase/Ibase;         % ohm, base resistance
wbase = 2*pi*fn;             % rad/s, base elec. radial frequency
wrated = 2*pi*1350 / 60;
Tbase = Sn/(wbase/p);        % N*m, base torque
Trated = Pn/(wbase/p);       % N.m  rated torque
Tscale = Tbase/Trated;       % Coefficient of base torque and rated torque
%Tbase = Pn/wrated;           % N*m, base mechanical torque
% Convert pu parameters back to ohmic values (for comparison / checks)
Rss = Rs*Zbase;  % ohm, stator resistance
Xls = Lls*Zbase; % ohm, stator leakage reactance
Rrr = Rr*Zbase;  % ohm, rotor resistance, referred to the stator side
Xlr = Llr*Zbase; % ohm, rotor leakage reactance, referred to the stator side
Xm = Lm*Zbase;   % ohm, magnetizing reactance

%% Control parameters
Ts = 1e-6;        % s, fundamental sample time
fsw = 20e3;       % Hz, switching frequency 
Tsc = 1/(fsw);    % s, control sample time

% Machine-side PI parameters
Kp_ids = 1.08;
Ki_ids = 207.58;
Kp_imr = 52.22;
Ki_imr = 2790.51;
Kp_iqs = 1.08;
Ki_iqs = 210.02;
Kp_wr = 65.47;
Ki_wr = 3134.24;
