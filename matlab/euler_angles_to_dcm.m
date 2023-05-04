function DCM = euler_angles_to_dcm(q)
	Rx = [1 0 0; 0 cos(q(1)) -sin(q(1)); 0 sin(q(1)) cos(q(1))];
	Ry = [cos(q(2)) 0 sin(q(2)); 0 1 0; -sin(q(2)) 0 cos(q(2))];
	Rz = [cos(q(3)) -sin(q(3)) 0; sin(q(3)) cos(q(3)) 0; 0 0 1];
	DCM = Rz * Ry * Rx;
end