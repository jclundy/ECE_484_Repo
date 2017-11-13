clear all; close all;
s = tf('s');
%% Inner loop parameters
Kp = 15;
p = 10.5;
z = 10;
T = 0.005; %sample time
K1 = 0.66538; %1.53
tau = 0.030541; %0.025
%% Inner loop controller
% define controller
C = Kp*(s+z)/(s+p);
% define the plant
P = K1/(tau*s^2 + s);
% system response
G = P*C/(1+P*C);
fb = bandwidth(G) * 0.5 / pi();
ts_recommended = 1/ (25 * fb);
% define inner loop digital controller
C_D = c2d(C,T,'tustin');
%% Plant Model
K2 = 0.062;
K3 = 4.78;
G_2 = K2 * K3 / s^2;
%% Outer loop parameters
p_c2 = 2.5; 
z_c2 = 0.35;
Kp_c2 = 7;
%% Outer loop controller
C_2 = Kp_c2 * (s + z_c2) / (s + p_c2);
% P_aug = G * G_2;
% G_outer = C_2 * P_aug / (1 + C_2 + P_aug);
% digital controller
C_D2 = c2d(C_2,T,'tustin')
a = C_D2.Numerator{1}(1); b =C_D2.Numerator{1}(2); 
c = C_D2.Denominator{1}(1); d= C_D2.Denominator{1}(2);

%% Graphing Results
sim('Lead_Filter_Continuous');
sim('Lead_Filter_Digital');
figure();
plot(ball_out); hold on;
plot(filtered_contol); hold on;
plot(ref_sig2);

title("System Response - Discrete Controller");
xlabel("Time (sec)");
ylabel("Ball Position (m)");
legend("Ball Position", "Input (for inner loop)", "Reference");

figure();
plot(ball_out_cont); hold on;
plot(filtered_contol_cont); hold on;
plot(ref_sig2);
title("System Response - Continuous Controller");
xlabel("Time (sec)");
ylabel("Ball Position  (m)");
legend("Ball Position", "Input (for inner loop)", "Reference");