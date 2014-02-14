function GaussianGenerator(basePath)

	x = (0 : 0.02 : 5)';
	
	% fitting trajectory
	s = 2.0;
	
	c2 = 0.2;
	c3 = 0.2;
	
	for c1=3:2:8
	
		for o=0.5:0.4:2.5
	
		%	y1 = c1 * exp(- (x - o) .^ 2 / s) + c1 * o / 8.0 * x;
		%	y1 = c1 * exp(- (x - o) .^ 2 / s) + c2 * sin(10 * x) + c1 * c3 * x;
			y1 = c1 * exp(- (x - o) .^ 2 / s) + c1 * c3 * x;
			
		%	dy1 = - 2 / s * c1 * (x + o) .* exp(- (x + o) .^ 2 / s);
		%	ddy1 = 2 / s^2 * c1 * exp(- (x + o) .^ 2 / s) .* (2 * o^2 + 4 * o .* x - s + 2 * x.^2);
			
			traj = [x, y1];
			
			qp = [o, c1];
			
		%	plot(x, y1);
			
		%	result = input('press key');
			
			save([basePath, '/query_', num2str(o), '_', num2str(c1), '.txt'], 'qp', '-ascii');
			save([basePath, '/traj_', num2str(o), '_', num2str(c1), '.txt'], 'traj', '-ascii');
			
		end
		
	end

end