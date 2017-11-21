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
C_inner = Kp*(s+z)/(s+p);
% define the plant
P_inner = K1/(tau*s^2 + s);
% system response
G_inner = P_inner*C_inner/(1+P_inner*C_inner);
fb = bandwidth(G_inner) * 0.5 / pi();
ts_recommended = 1/ (25 * fb);
% define inner loop digital controller
C_D = c2d(C_inner,T,'tustin');
%% Plant Model
K2 = 0.062;
K3 = 4.78;
P_outer = K2 * K3 / s^2;
%% Outer loop parameters
p_c2 = 2.5; 
z_c2 = 0.35;
Kp_c2 = 7;
%% Outer loop controller
C_outer = Kp_c2*(s + z_c2) / (s + p_c2) * 1/s;
P_aug = G_inner * P_outer;
G_outer = C_outer * P_aug / (1 + C_outer + P_aug);
% digital controller
C_D2 = c2d(C_outer,T,'tustin')
return
%% Graphing Results
sim('Lead_Filter_Integrator_Continuous_a');
sim('Lead_Filter_Integrator_Digital_a');
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