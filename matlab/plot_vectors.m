function plot_vectors(t_space, v1, v2, v3, titles, skip)
	subplot(2, 3, 1)
	hold on
	plot(t_space(1:skip:end), v3(1, 1:skip:end), 'r')
	plot(t_space(1:skip:end), v1(1, 1:skip:end), 'b')
	title(titles(1))
	
	subplot(2, 3, 4)
	plot(t_space(1:skip:end), v2(1, 1:skip:end))
	title(titles(4))
	
	subplot(2, 3, 2)
	hold on
	plot(t_space(1:skip:end), v3(2, 1:skip:end), 'r')
	plot(t_space(1:skip:end), v1(2,1:skip:end), 'b')
	title(titles(2))

	subplot(2, 3, 5)
	plot(t_space(1:skip:end), v2(2, 1:skip:end))
	title(titles(5))

	subplot(2, 3, 3)
	hold on
	plot(t_space(1:skip:end), v3(3, 1:skip:end), 'r')
	plot(t_space(1:skip:end), v1(3, 1:skip:end), 'b')
	title(titles(3))
	legend("reference", "feedback")

	subplot(2, 3, 6)
	plot(t_space(1:skip:end), v2(3, 1:skip:end))
	title(titles(6))
	
end