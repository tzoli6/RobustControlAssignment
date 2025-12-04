load("Assignment_Data_SC42145_2025.mat")

sys = ss(A, B, C, D);

FWT = tf(sys);

s = tf("s");

Gmimo = FWT(:, 1:2);
% Find out if if the distrubance input has to be there for the design


Kp = realp('Kp', ones(1, 4));
Ki = realp('Ki', ones(1, 4));
Kd = realp('Kd', ones(1, 4));
Tf = realp('Tf', ones(1, 4));
omega = realp('omega', 1);
beta_1 = realp('beta_1', 1);
beta_2 = realp('beta_2', 1);

Wp = [(s/2.4 + 1.885)/(s+1.885*10^-4) 0; 0 0.1]
Wp.u = {'e1', 'e2'};
Wp.y = {'z1_1', 'z1_2'};

Wu = [0.01 0;
    0 (5*10^(-3)*s^2+7*10^(-4)*s+5*10^(-5))/(s^2+14*10^(-4)*s+10^(-6))];
Wu.u = {'u1', 'u2'};
Wu.y = {'z2_1', 'z2_2'};

e = 0.01;
c1 = Kp(1) +  Ki(1)/(s+e) + (Kd(1) * s)/( Tf(1) * s + 1);
c2 = Kp(2) +  Ki(2)/(s+e) + (Kd(2) * s)/( Tf(2) * s + 1);
c3 = (s^2 + 2*omega*beta_1*s + omega^2)/(s^2 + 2*omega*beta_2*s + omega^2);
c4 = Kp(4) +  Ki(4)/(s+e) + (Kd(4) * s)/( Tf(4) * s + 1);

C_block = [c1 c2;
            c3 0];
C_block.Blocks

Gmimo = -1.*Gmimo;
Gmimo.u = {'u1','u2'};
Gmimo.y = {'y1','y2'};

C_block.u = {'e1', 'e2'};
C_block.y = {'u1', 'u2'};

sum1 = sumblk('e1 = r1 - y1');
sum2 = sumblk('e2 = r2 - y2');

full_plant = connect(Gmimo, Wp, Wu, C_block, sum1, sum2, ...
    {'r1', 'r2'}, ...
    {'y1', 'y2', 'z1_1', 'z1_2', 'z2_1','z2_2'});


opt = hinfstructOptions('Display', 'final', 'RandomStart', 5);
N_siso = hinfstruct (full_plant, opt)

Kp_opt = N_siso.Blocks.Kp.Value;
Kd_opt = N_siso.Blocks.Kd.Value;
Ki_opt = N_siso.Blocks.Ki.Value;
Tf_opt = N_siso.Blocks.Tf.Value;
omega_opt = N_siso.Blocks.omega.Value;
beta_1_opt = N_siso.Blocks.beta_1.Value;
beta_2_opt = N_siso.Blocks.beta_2.Value;

c1 = Kp_opt(1) + Ki_opt(1)/(s+e) + (Kd_opt(1) * s)/(Tf_opt(1)*s + 1)
c2 = Kp_opt(2) + Ki_opt(2)/(s+e) + (Kd_opt(2) * s)/(Tf_opt(2)*s + 1)
c3 = (s^2 + 2*omega_opt*beta_1_opt*s + omega_opt^2)/(s^2 + 2*omega_opt*beta_2_opt*s + omega_opt^2)
%c4 = Kp_opt(4) + Ki_opt(4)/(s+e) + (Kd_opt(4) * s)/(Tf_opt(4)*s + 1)

C_block = [c1 c2; 
           c3 0];


% Contstruct CL for simulation
G_sim = FWT(:, 1:2);
G_sim.u = 'u';
G_sim.y = 'y_{plant}';

K_sim = C_block;
K_sim.u = 'y_{meas}';
K_sim.y = 'u';

Gd_sim = FWT(: ,3) ;
Gd_sim.u = 'V';
Gd_sim.y = 'y_dist';

Sum_sim = sumblk('y_{meas} = y_{plant} + y_dist' , 2);
P_sim = connect ( G_sim, K_sim, Gd_sim, Sum_sim, ...
    { 'V' }, ...
    { 'y_{meas}', 'u' });

P_sim.InputName = { 'V( m/s)' };
% P_sim.OutputName = [G.OutputName; G.InputName];


fig = figure('Color','white','Position',[200 200 600 800]);

set(groot, 'defaultAxesFontSize', 14);
set(groot, 'defaultAxesLineWidth', 1.2);
set(groot, 'defaultAxesBox', 'on');
set(groot, 'defaultAxesXGrid','on');
set(groot, 'defaultAxesYGrid','on');
set(groot, 'defaultAxesGridAlpha', 0.20);
set(groot, 'defaultAxesGridLineStyle','-');

tl = tiledlayout(fig,'vertical','TileSpacing','compact','Padding','compact');
tl.InnerPosition = [0.02 0.02 0.96 0.96];
tl.OuterPosition = [0 0 1 1];

% Function to save tight figure
saveTight = @(filename) exportgraphics(fig, filename, ...
    'Resolution', 600, ...
    'BackgroundColor','white', ...
    'ContentType','image');

step(P_sim)
grid on;

saveTight("fixed_disturbance_sim.png")

I2 = eye(2);
L = G_sim * C_block;

S = feedback(I2, L);
T = I2 - S;

w = logspace(-3, 3, 500);

figure;
subplot(2,1,1)
bodemag(S, w), grid on
title('Sensitivity S(s)')

subplot(2,1,2)
bodemag(T, w), grid on
title('Complementary Sensitivity T(s)')