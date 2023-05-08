function [Kref, Kfb, Kff] = Rufous_nl_controller(Kt, m, g, q, gatt, grat, gq, gdq, gzp, gzd)
	n_inputs = 4;			% number of inputs
	state_dim = 3;			% dimension of each state
	tilt = abs(cos(q(1)) * cos(q(2)));
	tau_k = [1  0 -1  0;
	         0 -1  0  1;
		    -10 10  -10  10]; % scaled by ten to slow down the yaw control

	t_k = [0 -1 0 1;
	       -1 0 1 0;
		   10 10 10 10]; % scaled by ten to slow down the altitude control 

	z = [0; 0; 1];
	Z = [z z z z];
	I = eye(state_dim);
	O = zeros(state_dim, state_dim);

	hover_throttle = sqrt(g * m / (Kt * n_inputs)) * tilt;
	
	Katt = gatt * pinv(tau_k);
	Krate = grat * pinv(tau_k);
	Kq = gq * [0 -1  0;
			   1  0  0;
			   0  0  0];
	Kdq = gdq * [0 -1  0;
			     1  0  0;
			     0  0  0];
	Kzp = gzp * Z';
	Kzd = gzd * Z';
	Kref = [I   O   O   O;
		    O   I   O   O;
		    Kq  Kdq I   O;
			O   O   O   I];
	Kfb = [Kzp Kzd Katt Krate];
	Kff = hover_throttle * ones(n_inputs, 1);
end
