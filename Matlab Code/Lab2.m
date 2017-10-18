close all;
Kp = 15;
p = 10.5;
z = 10;
T = 0.005; %sample time
K1 = 0.66538; %1.53
tau = 0.030541; %0.025

s = tf('s');
% define controller
C = Kp*(s+z)/(s+p);
% define the plant
P = K1/(tau*s^2 + s);
% system response
G = P*C/(1+P*C);
fb = bandwidth(G) * 0.5 / pi();
ts_recommended = 1/ (25 * fb);

C_D = c2d(C,T,'tustin')

sim('Lab2_Model');
sim('Lab2_Model_Continuous');
figure();
plot(theta_out); hold on;
plot(theta_out_cont);
plot(input_sig);


title("System Response - Discrete Controller");
xlabel("Time (sec)");
ylabel("Gear RotationAngle (rad)");
legend("Discrete", "Continuous", "Input");



figure();
plot(theta_out_cont); hold on;
plot(input_sig_cont);
title("System Response - Continuous Controller");
xlabel("Time (sec)");
ylabel("Gear RotationAngle (rad)");
legend("Output", "Input");