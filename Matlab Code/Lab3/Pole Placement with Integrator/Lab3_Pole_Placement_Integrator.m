close all;
%% Inner loop parameters
Kp = 15;
p = 10.5;
z = 10;
T = 0.005; %sample time
K1 = 0.66538; %1.53
tau = 0.030541; %0.025
%% Inner loop controller
s = tf('s');
% define controller
C = Kp*(s+z)/(s+p);
% define the plant
P = K1/(tau*s^2 + s);
% system response
G = P*C/(1+P*C);
fb = bandwidth(G) * 0.5 / pi();
ts_recommended = 1/ (25 * fb);
C_D = c2d(C,T,'tustin') % for discrete controller
%% Outer loop parameters
K2 = 0.062;
K3 = 4.78;
G_2 = K2 * K3 / s^2;
% Augmented plant of order 3
G_aug = G_2 * 1/s;
%% Outer Loop Pole placement controller
% Time parameters
OS = 0.45;
Ts = 7;
ln_os = log(0.45);
zeta = -ln_os / sqrt(pi()^2 + ln_os^2);
omega = 4 / (zeta*Ts);
theta = cos(zeta);
% Determine pole placement regions
real_value = zeta*omega;
imaginary_value = real_value * tan(theta);
beta = imaginary_value * 0.05;
alpha = real_value + 0.01;
k = alpha * 25;
% Char poly of order 2n - 1 = 2(3) - 1 = 5
% Characteristic polynomial with desired poles
char_poly = (s+ alpha + 1i*beta)*(s+ alpha - 1i*beta)*(s+k)*(s+k)*(s+k);
cp_num = char_poly.Numerator{1};
% Extracting coefficients of the desired polynomial
C = cp_num(:);
% Setting up system of equations to solve for the controller coefficients
P_num = G_aug.Numerator{1};
P_den = G_aug.Denominator{1};
b3 =0; b2 = 0; b1 = 0; b0 = K2 * K3;
a3=1; a2 = 0; a1 = 0; a0 = 0;
a_vec = [a3;a2;a1;a0]
b_vec = [b3;b2;b1;b0];
A = [a3 0  0  b3 0  0 ;
     a2 a3 0  b2 b3 0 ;
     a1 a2 a3 b1 b2 b3;
     a0 a1 a2 b0 b1 b2;
     0  a0 a1 0  b0 b1;
     0  0  b0 0  0  b0;];
F = inv(A)*C
% % Extracting controller denominator
f2 = F(1); f1 = F(2); f0 = F(3);
% Extracting controller numerator
g2 = F(4); g1 = F(5); g0 = F(6);
% Define Pole Placement Controller 
C_2 = (g2*s^2 + g1*s + g0)/(f2*s^2 + f1*s + f0)

% Determine controller gain
DC_gain = evalfr(C_2,0);
Kp_c2 = 2 / DC_gain;
C2_aug = C_2 * 1/s
C2aug_num = C2_aug.Numerator{1}; C2aug_den = C2_aug.Denominator{1};
% 1/(4*DC_gain);
%% Digital Controller
C_D2 = c2d(C2_aug, T, 'tustin')

%% Digital Controller Difference Equation
syms e0 e1 e2 e3 u0 u1 u2 u3
cd_num = C_D2.Numerator{1}; cd_den = C_D2.Denominator{1};
p3 = cd_num(1); p2 = cd_num(2); p1 = cd_num(3); p0 = cd_num(4);
q3 = cd_den(1); q2 = cd_den(2); q1 = cd_den(3); q0 = cd_den(4);
eqn_coeff = [p3 p2 p1 p0 q2 q1 q0]/q3
e0 = vpa(eqn_coeff * [e0; e1; e2; e3; u1; u2; u3],5)

%% Graphing Results
sim('Pole_Placement_Integrator_Continuous');
sim('Pole_Placement_Integrator_Digital');
% Graph Continuous
figure();
plot(ball_out_cont); hold on;
plot(filtered_contol_cont);
% plot(input_sig_cont); hold on;
plot(ref_sig2_cont);
title("System Response - Continuous Controller");
xlabel("Time (sec)");
ylabel("Ball Position  (m)");
legend("Ball Position", "Input (for inner loop)", "Reference");

os = max(ball_out_cont);
percent_os = (os-0.18)/(0.18-0.15)*100

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