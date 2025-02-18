%	Approximated based on hardware (DJI F-450 frame w/ 10in propellers) 
l = 0.23;
g = 9.81;
Kt = 0.02;
mass = 0.025;
Ixx = 4e-3;
Iyy = 4e-3;
Izz = 7e-3;
Ib = diag([Ixx Iyy Izz]);
ht = sqrt(g * mass / (Kt * 4));

RUFOUS_N_INPUTS = 4;			% number of inputs
RUFOUS_N_STATES = 6;			% number of states
RUFOUS_STATE_DIM = 3;			% dimension of each state

I = eye(RUFOUS_STATE_DIM);
O = zeros(RUFOUS_STATE_DIM, RUFOUS_STATE_DIM);

tau_k = [1  0 -1  0;		% Alter this to effect the sensitivity of input -> state channels
	     0 -1  0  1;
		-1  1 -1  1];

A = [O I;				    % block size (state_dim, state_dim)
	 O O];
B = Kt *[zeros(RUFOUS_STATE_DIM, RUFOUS_N_INPUTS); Ib^-1 * l * tau_k * 2 * ht];		% block size (state_dim, n_inputs) ignore non-controlled forces
C = eye(RUFOUS_N_STATES);
D = 0;

G = ss(A, B, C, D);

freq_range = [1e-10 1e4];

O = zeros(RUFOUS_N_STATES / 2, RUFOUS_N_STATES / 2);
Wq = makeweight(1e4, [1e4, 1], 1e-5) * eye(RUFOUS_N_STATES / 2);
Wdq = makeweight(1e1, [1e7, 1], 1e-5) * eye(RUFOUS_N_STATES / 2);
WP = [Wq O; O Wdq];

systemnames = 'G WP';
inputvar = '[qr(6); u(4)]';
outputvar = '[WP; qr-G]';
input_to_G = '[u]';
input_to_WP = '[qr-G]';
sysoutname = 'P';
sysic;
P_nominal = minreal(ss(P));

% In rufous_h2 (line 44) 
% Warning: GAM=Inf because the closed-loop system has a nonzero feedthrough from w to z.
% Returning the optimal H2 controller K when ignoring this feedthrough. 
% Warning: GAM=Inf because the closed-loop system has a nonzero feedthrough from w to z.
% Returning the optimal H2 controller K when ignoring this feedthrough. 
% Warning: GAM=Inf because the closed-loop system has a nonzero feedthrough from w to z.
% Returning the optimal H2 controller K when ignoring this feedthrough. 

% Z is error, for tracking problem, maybe move this to the output

[K, ~, ~] = h2syn(P, 6, 4);

L = K * G;
Si = (eye(RUFOUS_N_INPUTS) + L)^-1;
Ti = L * Si;

L = G * K;
So = (eye(RUFOUS_N_STATES) + L)^-1;
To = L * So;

figure, hold on
h = sigmaplot(G, 'b', L, 'r');
plot(freq_range, [1, 1], 'k');
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log');
title("Open loop")
legend("G", "L")

figure, hold on
h = sigmaplot(Ti, 'b', WP, 'r--');
plot(freq_range, [1, 1], 'k');
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log');
title("T vs WP")

figure, hold on
h = sigmaplot(Si, 'b', Si * K, 'g', WP^-1, 'r--');
plot(freq_range, [1, 1], 'k');
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log');
title("S vs KS")
legend("S", "KS", "WP")

figure, hold on
h = sigmaplot(Ti);
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log');
title("Robust Stability")

figure, hold on
h = sigmaplot(WP * (eye(RUFOUS_N_STATES) - To));
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log');
title("Robust Performance")

figure
step(T(:, 1))
title("Step roll angle reference")
figure
step(T(:, 2))
title("Step pitch angle reference")
figure
step(T(:, 3))
title("Step yaw angle reference")

figure
step(S(:, 1))
title("Step roll angle disturbance")
figure
step(S(:, 2))
title("Step pitch angle disturbance")
figure
step(S(:, 3))
title("Step yaw angle disturbance")


% the solver throws some warnings but seems fine,
% Achieves good performance using the same weights
% as the Hinf controller. Almost identical to Hinf
