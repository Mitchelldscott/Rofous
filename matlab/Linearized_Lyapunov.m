function V = Linearized_Lyapunov(rz, vx, vy, vz, wx, wy, wz, mass, Ib)
	V = (0.5 * (Ib(1,1) * wx.^2) + (Ib(2,2) * wy.^2 + (Ib(3,3) * wz.^2))); %+ (0.5 * mass * (vx.^2 + vy.^2 + vz.^2)) - (9.81 * mass * rz);
end