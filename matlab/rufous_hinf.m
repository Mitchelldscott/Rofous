close all; clear
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

WP = makeweight(0.001, [100, 1], 10) * eye(RUFOUS_N_STATES);

systemnames = 'G WP';
inputvar = '[qr(6); u(4)]';
outputvar = '[WP; qr-G]';
input_to_G = '[u]';
input_to_WP = '[qr-G]';
sysoutname = 'P';
sysic;
P_nominal = minreal(ss(P));

[K, ~, ~] = hinfsyn(P_nominal, 6, 4);

L = G * K;
S = 1 / (eye(RUFOUS_N_STATES) + L);
T = L * S;

figure, hold on
h = sigmaplot(G, 'b', L, 'r');
plot(freq_range, [1, 1], 'k');
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log');
title("Open loop")
legend("G", "L")

figure, hold on
h = sigmaplot(T, 'b', WP^-1, 'r--');
plot(freq_range, [1, 1], 'k');
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log');
title("T vs WP^{-1}")

figure, hold on
h = sigmaplot(S, 'b', K * S, 'r--');
plot(freq_range, [1, 1], 'k');
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log');
title("S vs KS")
legend("S", "KS")

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

% This controller is crazy conservative, might make sense that slowing down
% the system stabilizes it. Also in reality this is an unstable operating
% point, being slow is much safer. Though this controller is so slow it
% would be unsusable in practice. Next time use mixsyn and have a better
% plan for what the 'loop' needs to look like (i.e. what channels and frequencies the
% controller should slow down/speed up).