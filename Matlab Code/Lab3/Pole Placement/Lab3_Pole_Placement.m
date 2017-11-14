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
%% Outer Loop Pole placement controller
% Time parameters
OS = 0.45;
Ts = 7;
ln_os = log(0.45);
zeta = -ln_os / sqrt(pi()^2 + ln_os^2);
omega = 4 / (zeta*Ts);
theta = cos(zeta);
% Determine pole placement regions
real_value = zeta*omega
imaginary_value = real_value * tan(theta)
% beta = imaginary_value * 0.4;
beta = imaginary_value * 0.5;
alpha = real_value + 0.01;
k = alpha * 25;
% Characteristic polynomial with desired poles
char_poly = (s+ alpha + 1i*beta)*(s+ alpha - 1i*beta)*(s+k)
% char_poly = (s+6+1i)*(s+6-1i)*(s+50);
% Extracting coefficients of the desired polynomial
c3 = char_poly.Numerator{1}(1);
c2 = char_poly.Numerator{1}(2);
c1 = char_poly.Numerator{1}(3);
c0 = char_poly.Numerator{1}(4);
% Setting up system of equations to solve for the controller coefficients
C = [c3; c2; c1; c0];
b2 = 0; b1 = 0; b0 = K2 * K3;
a2 = 1; a1 = 0; a0 = 0;
A = [a2 0  b2 0 ; 
     a1 a2 b1 b2; 
     a0 a1 b0 b1; 
     0  a0 0  b0]
F = inv(A)*C
% % Extracting controller denominator
f1 = F(1); f0 = F(2);
% Extracting controller numerator
g1 = F(3); g0 = F(4);
% Define Pole Placement Controller 
C_2 = (g1*s + g0)/(f1*s + f0)
% Determine controller gain
DC_gain = evalfr(C_2,0);
Kp_c2 = 1 / DC_gain;
% 1/(4*DC_gain);
%% Digital Controller
C_D2 = c2d(C_2, T, 'tustin');
a = C_D2.Numerator{1}(1); b =C_D2.Numerator{1}(2); 
c = C_D2.Denominator{1}(1); d= C_D2.Denominator{1}(2);
format long
CD_NUM = [a, b]
format long
CD_DEN = [c, d]
%% Digital Controller Difference Equation
% u[k] = -d/c u[k-1] + a/c r[k] + b/c r[k-1]
% u[k] = f1 u[k-1] + f2 r[k] + f3 r[k-1]
diff1 = -d/c; diff2 = a/c; diff3 = b/c;
[diff1; diff2; diff3]
%% Outer Loop Closed Loop Response
P_aug = G * G_2;
G_outer = C_2 * P_aug / (1 + C_2 + P_aug);

%% Graphing Results
sim('Pole_Placement_Continuous_a'); % continuous model
sim('Pole_Placement_Digital_a');
figure();
plot(ball_out); hold on;
plot(filtered_contol); %saturated motor voltage
plot(ref_sig2);
hold off;

title("System Response - Discrete Controller");
xlabel("Time (sec)");
ylabel("Ball Position (m)");
legend("Ball Position", "Saturated Angle Input (for inner loop)", "Reference");

% figure();
% plot(ball_out_cont); hold on;
% plot(filtered_contol_cont); hold on;
% plot(ref_sig2_cont);
% title("System Response - Continuous Controller");
% xlabel("Time (sec)");
% ylabel("Ball Position  (m)");
% legend("Ball Position", "Input (for inner loop)", "Reference");