function Taup = Rufous_Torque(u, Kt, l)
	Taup = Kt * l * [1 0 -1 0; 0 -1 0 1; -1 1 -1 1] * u.^2;
end