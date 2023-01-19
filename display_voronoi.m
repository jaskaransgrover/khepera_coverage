function h = display_voronoi(X,Y,ho)
if(ho~=1)
    delete(ho);
end
h = voronoi(X,Y);
for i = 1:length(h)
 h(i).LineWidth = 4;
 h(i).Color = 'k';
 hold on;
end


end