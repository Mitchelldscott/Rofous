function V = Rufous_Lyapunov_dot(q1, q2, q3, w1, w2, w3, dw1, dw2, dw3, Ib, Katt)
	V_omega = (Ib(1,1) * w1 .* dw1) + (Ib(2,2) * w2 .* dw2) + (Ib(3,3) * w3 .* dw3);
	gamma_att = [Katt(1,1)^2 Katt(4, 2)^2 2*Katt(2, 3)^2];
	V_krat = (gamma_att(1) * q1 .* w1) + (gamma_att(2) * q2 .* w2) + (gamma_att(3) * q3 .* w3);
	V = V_omega + V_krat;
end