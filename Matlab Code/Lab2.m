Kp = 15;
p = 10.5;
z = 10;
T = 0.005; %sample time
K1 = 0.66538; %1.53
tau = 0.030541; %0.025

s = tf('s');
C = Kp*(s+z)/(s+p);
C_D = c2d(C,0.005,'tustin')

sim('Lab2_Model');

figure();
plot(theta_out); hold on;
plot(input_sig);

title("Saturator Simulation");
xlabel("Time (sec)");
ylabel("Gear RotationAngle (rad)");
legend("Output", "Input");