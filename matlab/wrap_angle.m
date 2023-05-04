function angle = wrap_angle(a)
	angle = a;
	for i = 1:size(a)
		if abs(a(i)) > 2 * pi
			angle(i) = -a(i);
		end

		while angle(i) > 2 * pi
			angle(i) = angle(i) - (2 * pi);
		end

		while angle(i) < -2 * pi
			angle(i) = angle(i) + (2 * pi);
		end

	end
end