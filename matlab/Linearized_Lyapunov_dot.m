function V = Linearized_Lyapunov_dot(wx, wy, wz, dwx, dwy, dwz, mass, Ib)
	V = ((Ib(1,1) * wx .* dwx) + (Ib(2,2) * wy .* dwy) + (Ib(3,3) * wz .* dwz));% + (0.5 * mass * (vx.^2 + vy.^2 + vz.^2)) - (9.81 * mass * rz);
end