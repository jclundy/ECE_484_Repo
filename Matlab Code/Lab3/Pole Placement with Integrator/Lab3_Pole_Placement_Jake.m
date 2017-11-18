close all;
%% Signal Input Parameters
enable_lead = 1;
% part c
Amplitude = 0.015;
Offset = 0.165;

%part d
Amplitude = 0.075;
Offset = 0.175;

if enable_lead == 1
    Amplitude = 0.05;
    Offset = 0.2;
end

%% Inner loop parameters
Kp = 15;
p = 10.5;
z = 10;
T = 0.005; % sample time chosen from Lab 1
K1 = 0.66538; % calculated from Lab 1
tau = 0.030541; % calculated from Lab 1
s = tf('s');

%% Inner loop controller
% define controller
C_inner = Kp*(s+z)/(s+p);
C_D_inner = c2d(C_inner,T,'tustin');
[C_num_inner, C_den_inner] = tfdata(C_D_inner, 'v');
% define the plant
P_inner = K1/(tau*s^2 + s);

%% Outer loop parameters
K2 = 0.062;
K3 = 4.78;
G_2 = K2 * K3 / s^2;
% Augmented plant of order 3
G_aug = G_2 * 1/s;

%% Outer loop lead parameters
if enable_lead == 1
    p_c2 = 2.5; 
    z_c2 = 0.35;
    Kp_c2 = 7;
    C_lead = Kp_c2 * (s + z_c2) / (s + p_c2);
    C_D_lead = c2d(C_lead, T, 'tustin');
    [C_num_outer, C_den_outer] = tfdata(C_D_lead, 'v');
end

%% Outer Loop Pole placement controller
% Time parameters
OS = 0.05;  % reduced from 45% to 5%, more reasonable
Ts = 7;     % from spec
ln_os = log(OS);
zeta = -ln_os / sqrt(pi()^2 + ln_os^2);

% Determine pole placement regions
real_value = 4 / Ts;
imaginary_value = real_value * tan(acos(zeta));
beta = imaginary_value - 0.05;
alpha = real_value + 0.01;
k = real_value * 10;
% Char poly of order 2n - 1 = 2(3) - 1 = 5
% Characteristic polynomial with desired poles
char_poly = (s+ alpha + 1i*beta)*(s+ alpha - 1i*beta)*(s+k)*(s+k)*(s+k);

% Setting up system of equations to solve for the controller coefficients
b3 = G_aug.Numerator{1}(1); b2 = G_aug.Numerator{1}(2); 
b1 = G_aug.Numerator{1}(3); b0 = G_aug.Numerator{1}(4);
a3 = G_aug.Denominator{1}(1); a2 = G_aug.Denominator{1}(2); 
a1 = G_aug.Denominator{1}(3); a0 = G_aug.Denominator{1}(4);
A = [a3 0  0  b3 0  0 ;
     a2 a3 0  b2 b3 0 ;
     a1 a2 a3 b1 b2 b3;
     a0 a1 a2 b0 b1 b2;
     0  a0 a1 0  b0 b1;
     0  0  a0 0  0  b0;];
F = inv(A)*cell2mat(char_poly.Numerator)';

% % Extracting controller denominator
f2 = F(1); f1 = F(2); f0 = F(3);
% Extracting controller numerator
g2 = F(4); g1 = F(5); g0 = F(6);
% Define Pole Placement Controller 
C_outer = (g2*s^2 + g1*s + g0)/(f2*s^2 + f1*s + f0);

% Determine controller gain
% DC_gain = evalfr(C_outer,0)
% Kp_c2 = 2 / DC_gain;
C_outer_aug = C_outer * 1/s;
%C2aug_num = C2_aug.Numerator{1}; C2aug_den = C2_aug.Denominator{1};
% 1/(4*DC_gain);

%% Digital Controller
if enable_lead == 0
    C_D_outer = c2d(C_outer_aug, T, 'tustin');
    [C_num_outer, C_den_outer] = tfdata(C_D_outer, 'v');


%% Digital Controller Difference Equation
    syms e0 e1 e2 e3 u0 u1 u2 u3
    cd_num = C_D_outer.Numerator{1}; cd_den = C_D_outer.Denominator{1};
    p3 = cd_num(1); p2 = cd_num(2); p1 = cd_num(3); p0 = cd_num(4);
    q3 = cd_den(1); q2 = cd_den(2); q1 = cd_den(3); q0 = cd_den(4);
    eqn_coeff = [p3 p2 p1 p0 -q2 -q1 -q0]/q3;
    u0 = vpa(eqn_coeff * [e0; e1; e2; e3; u1; u2; u3],5);
end

%% Graphing Results
%sim('Pole_Placement_Integrator_Continuous');
sim('Pole_Placement_Jake');
% Graph Continuous
% figure();
% plot(ball_out_cont); hold on;
% plot(filtered_contol_cont);
% % plot(input_sig_cont); hold on;
% plot(ref_sig2_cont);
% title("System Response - Continuous Controller");
% xlabel("Time (sec)");
% ylabel("Ball Position  (m)");
% legend("Ball Position", "Input (for inner loop)", "Reference");

percent_os = (max(ball_out)-0.18)/(0.18-0.15)*100;

% Graph digital
figure();
plot(ball_out); hold on;
plot(filtered_contol);%saturated motor voltage
% plot(input_sig); 
plot(ref_sig2);
hold off;
title("System Response - Discrete Controller");
xlabel("Time (sec)");
ylabel("Ball Position (m)");
legend("Ball Position", "Saturated Angle Input (for inner loop)", "Reference");