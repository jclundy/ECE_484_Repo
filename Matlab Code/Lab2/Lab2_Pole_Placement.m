close all;
Kp = 15;
p = 10.5;
z = 10;
T = 0.005; %sample time
K1 = 0.66538; %1.53
tau = 0.030541; %0.025

clear all; close all;
s = tf('s')
T = 0.005;
Kp = 1;
p = 10.5;
z = 10;
T = 0.005; %sample time
K1 = 0.66538; %1.53
tau = 0.030541; %0.025
OS = 0.05;
Ts = 0.5;
ln_os = log(0.45);
zeta = -ln_os / sqrt(pi()^2 + ln_os^2);
omega = 4 / (zeta*Ts);
theta = cos(zeta);
% Determine pole placement regions
real_value = zeta*omega
imaginary_value = omega * sqrt(1-zeta^2)
% Characteristic polynomial with desired poles
alpha = real_value + 1;
beta = imaginary_value - 4;
char_poly = (s+ alpha + 1i*beta)*(s+ alpha - 1i*beta)*(s+15*real_value)
% Extracting coefficients of the desired polynomial
c3 = char_poly.Numerator{1}(1);
c2 = char_poly.Numerator{1}(2);
c1 = char_poly.Numerator{1}(3);
c0 = char_poly.Numerator{1}(4);
C = [c3; c2; c1; c0];
% Setting up system of equations to solve for the controller coefficients
% P(s) = (b2*s^2 + b1*s + b0) / (a2*s^2 + a1*s + a0)
b2 = 0; b1 = 0; b0 = K1;
a2 = tau; a1 = 1; a0 = 0;
A = [a2 0  b2 0 ; 
     a1 a2 b1 b2; 
     a0 a1 b0 b1; 
     0  a0 0  b0]
F = inv(A)*C
% Extracting controller zeroes
f1 = F(1); f0 = F(2);
% Extracting controller poles
g1 = F(3); g0 = F(4);
% Define Pole Placement Controller 
C = (g1*s + g0)/(f1*s + f0)

C_D = c2d(C,T,'tustin')

% sim('Lab2_Model');
sim('Lab2_Pole_Placement_Continuous');
figure();
% plot(theta_out); hold on;
plot(theta_out_cont);
plot(input_sig_cont);


title("System Response - Discrete Controller");
xlabel("Time (sec)");
ylabel("Gear RotationAngle (rad)");
% legend("Discrete", "Continuous", "Input");
legend("Continuous", "Input");


figure();
plot(theta_out_cont); hold on;
plot(input_sig_cont);
title("System Response - Continuous Controller");
xlabel("Time (sec)");
ylabel("Gear RotationAngle (rad)");
legend("Output", "Input");