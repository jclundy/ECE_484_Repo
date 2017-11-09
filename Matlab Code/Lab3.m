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
p_c2 = 2.5;
z_c2 = 0.35;
Kp_c2 = 7;
K2 = 0.062;
K3 = 4.78;
%% Outer loop controller
% define controller
C_2 = Kp_c2 * (s + z_c2) / (s + z_c2);
% define the plant
G_2 = K2 * K3 / s^2;
P_aug = G * G_2;
G_outer = C_2 * P_aug / (1 + C_2 + P_aug);

%% Graphing Results
sim('Lab3_Model_Continuous'); % continuous model
% sim('Lab3_Model_Discrete');
figure();
% plot(ball_out); hold on;
plot(ball_out_cont);
plot(input_sig);


title("System Response - Discrete Controller");
xlabel("Time (sec)");
ylabel("Ball Position (m)");
legend("Discrete", "Continuous", "Input");

% figure();
% plot(theta_out_cont); hold on;
% plot(input_sig_cont);
% title("System Response - Continuous Controller");
% xlabel("Time (sec)");
% ylabel("Ball Position  (m)");
% legend("Output", "Input");