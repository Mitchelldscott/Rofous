function [dr, ddr, dq, do] = Rufous_Dynamics(rdot, q, omega, u, Kt, m, g, l, I)
	% constrain the input to a feasble one
	u = max(u, [0; 0; 0; 0]);

	dr = rdot;
	ddr = (Rufous_Thrust(q, u, Kt) - [0; 0; m * g]) / m;

	dq = omega;
	do = I^-1 * (-tilde(omega) * I * omega + Rufous_Torque(u, Kt, l));
end