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

Wp = [(s/2.4 + 1.885)/(s+1.885*10^-4) 0; 0 0.1]
Wp.u = {'e1', 'e2'};
Wp.y = {'z1_1', 'z1_2'};

Wu = [0.01 0;
    0 (5*10^(-3)*s^2+7*10^(-4)*s+5*10^(-5))/(s^2+14*10^(-4)*s+10^(-6))];
Wu.u = {'u1', 'u2'};
Wu.y = {'z2_1', 'z2_2'};


c1 = Kp(1) +  (Kd(1) * s)/( Tf(1) * s + 1);
c2 = Kp(2) +  (Kd(2) * s)/( Tf(2) * s + 1);
c3 = Kp(3) +  (Kd(3) * s)/( Tf(3) * s + 1);
c4 = Kp(4) +  (Kd(4) * s)/( Tf(4) * s + 1);

C_block = [c1 c2; c3 c4];
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
N_siso = hinfstruct (full_plant, opt);

Kp_opt = N_siso.Blocks.Kp.Value;
Kd_opt = N_siso.Blocks.Kd.Value;
Tf_opt = N_siso.Blocks.Tf.Value;

c1 = Kp_opt(1) + (Kd_opt(1) * s)/(Tf_opt(1)*s + 1);
c2 = Kp_opt(2) + (Kd_opt(2) * s)/(Tf_opt(2)*s + 1);
c3 = Kp_opt(3) + (Kd_opt(3) * s)/(Tf_opt(3)*s + 1);
c4 = Kp_opt(4) + (Kd_opt(4) * s)/(Tf_opt(4)*s + 1);

C_block = [c1 c2; c3 c4];


% Contstruct CL for simulation
G_sim = FWT(:, 1:2);
G_sim.u = 'u';
G_sim.y = 'y_plant';

K_sim = C_block;
K_sim.u = 'y_meas';
K_sim.y = 'u';

Gd_sim = FWT(: ,3) ;
Gd_sim.u = 'V';
Gd_sim.y = 'y_dist';

Sum_sim = sumblk('y_meas = y_plant + y_dist' , 2);
P_sim = connect ( G_sim, K_sim, Gd_sim, Sum_sim, ...
    { 'V' }, ...
    { 'y_meas', 'u' });

P_sim.InputName = { 'V( m/s)' };
P_sim.OutputName = [G.OutputName; G.InputName];
step(P_sim)

