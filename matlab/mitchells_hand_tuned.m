close all; clear
%	Approximated based on hardware (DJI F-450 frame w/ 10in propellers) 
l = 0.23;
g = 9.81;
Kt = 0.02;				% uncertainty here would make things difficult
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
		-1  1 -1  1];       % adjust this row in the controller to tune Z sensitivity (speed up decrease values/slow down increase values)

A = [O I;				    % block size (state_dim, state_dim)
	 O O];
B = Kt *[zeros(RUFOUS_STATE_DIM, RUFOUS_N_INPUTS); Ib^-1 * l * tau_k * 2 * ht];		% block size (state_dim, n_inputs) ignore non-controlled forces
C = eye(RUFOUS_N_STATES);
D = 0;

G = ss(A, B, C, D);

freq_range = [1e-10 1e4];


Katt = 150 * pinv(tau_k);
Krate = 25 * pinv(tau_k);
K = [Katt Krate];

L = G * K;
S = 1 / (eye(RUFOUS_N_STATES) + L);
T = L * S;

O = zeros(RUFOUS_N_STATES / 2, RUFOUS_N_STATES / 2);
Wq = makeweight(1e4, [1e4, 1], 1e-5) * eye(RUFOUS_N_STATES / 2);
Wdq = makeweight(1e1, [1e7, 1], 1e-5) * eye(RUFOUS_N_STATES / 2);
WP = [Wq O; O Wdq];

figure, hold on
h = sigmaplot(G(1, :), G(2, :), G(3, :), L(1, :), L(2, :), L(3, :), linspace(freq_range(1), freq_range(2))); 
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log')
plot(freq_range, [1, 1], 'k');
title("Singular value of Decoupled OL system (inputs -> attitude)")
legend("\theta_G", "\phi_G", "\psi_G", "\theta_L", "\phi_L", "\psi_L")

figure, hold on
h = sigmaplot(G(4, :), G(5, :), G(6, :), L(4, :), L(5, :), L(6, :), linspace(freq_range(1), freq_range(2)));
setoptions(h, 'MagUnits', 'abs', 'MagScale', 'log')
plot(freq_range, [1, 1], 'k');
title("Singular value of Decoupled OL system (inputs -> angular rate)")
legend("\omega_xG", "\omega_yG", "\omega_zG", "\omega_xL", "\omega_yL", "\omega_zL")

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

% This is the best performing controller, the only one that could achieve
% somewhat desirable performance. The hinf controller is too conservative
% and slow, the H2 controller doesn't really work, like at all. Uncertainty
% should be added to Kt the thrust constant to demonstrate with imperfect
% thrust (input to G) the controller can remain near a reference.