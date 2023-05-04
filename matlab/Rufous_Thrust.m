function Tp = Rufous_Thrust(q, u, Kt)
	Rnb = euler_angles_to_dcm(-q);
	z_hat_b = Rnb \ [0; 0; 1];
	Tp = Kt * sum(u.^2) * z_hat_b;
end