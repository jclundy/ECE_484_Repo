OS = 0.45;
Ts = 7;
ln_os = log(0.45);
zeta = -ln_os / sqrt(pi()^2 + ln_os^2)
omega = 4 / (zeta*Ts)

real_value = zeta*omega
theta = cos(zeta)

imaginary_value = omega * sqrt(1-zeta^2)
K2 = 0.062;
K3 = 4.78;
b2 = 0;
b1 = 0;
b0 = K2 * K3;
a2 = 1;
a1 = 0;
a0 = 0;
s = tf('s')
char_poly = (s+ 0.5124 + j*2.2482)*(s+ 0.5124 - j*2.2482)*(s+6)
c3 = 1;
c2 = 7.025;
c1 = 11.47;
c0 = 31.9;
C = [c3; c2; c1; c0];
A = [a2 0 b2 0 ; a1 a2 b1 b2; a0 a1 b0 b1; 0 a0 0 b0]
F = inv(A)*C
g1 = F(1) 
g0 = F(2)
f1 = F(3) 
f0 = F(4)
C_t = (g1*s + g0)/(f1*s + f0)
%Sampling time
T = 0.005;

C_d = c2d(C_t, T, 'tustin');
a = C_D.Numerator{1}(1); b =C_D.Numerator{1}(2); 
c = C_D.Denominator{1}(1); d= C_D.Denominator{1}(2);
% converting discrete controller from z-domain to discrete time
% domain:
% u[k] = -d/c u[k-1] + a/c r[k] + b/c r[k-1]
% u[k] = f1 u[k-1] + f2 r[k] + f3 r[k-1]
f1 = -d/c; f2 = a/c; f3 = b/c;
[f1 f2 f3]