function visualize_dist_area(Dist_distribution)
s = pcolor(Dist_distribution.X,Dist_distribution.Z,Dist_distribution.intensity);
Nmap = 100;
map =[ones(Nmap*2,1) [ones(Nmap,1) linspace(1,0,Nmap)'; linspace(1,0,Nmap)' zeros(Nmap,1)]];
colormap(map)
%  s.LineWidth = 0;
s.EdgeColor = 'none';
s.FaceColor = 'interp';
end