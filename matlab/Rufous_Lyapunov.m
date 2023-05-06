function V = Rufous_Lyapunov(qx, qy, qz, wx, wy, wz, qr, Ib, Katt)
	V_ang = (Ib(1,1) * wx.^2) + (Ib(2,2) * wy.^2) + (Ib(3,3) * wz.^2);

	V_att_x = (Katt(1, 1) * (qr(1) - qx)).^2 + (Katt(2, 1) * (qr(1) - qx)).^2 + (Katt(3, 1) * (qr(1) - qx)).^2 + (Katt(4, 1) * (qr(1) - qx)).^2;
	V_att_y = (Katt(1, 2) * (qr(2) - qy)).^2 + (Katt(2, 2) * (qr(2) - qy)).^2 + (Katt(3, 2) * (qr(2) - qy)).^2 + (Katt(4, 2) * (qr(2) - qy)).^2;
	V_att_z = (Katt(1, 3) * (qr(3) - qz)).^2 + (Katt(2, 3) * (qr(3) - qz)).^2 + (Katt(3, 3) * (qr(3) - qz)).^2 + (Katt(4, 3) * (qr(3) - qz)).^2;

	V = 0.5 * (V_ang + V_att_x + V_att_y + V_att_z);
end