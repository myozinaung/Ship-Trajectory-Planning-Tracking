function plotCircleSeries(X, Y, PSI, rad)
%% Ship Dimension

point = -1.25:0.5:1.25;

for i = 1:length(point)
    xposP = X + point(i).*cos(PSI);
    yposP = Y + point(i).*sin(PSI);
    drawCircle([xposP, yposP, rad]); hold on;
end

axis equal;

end
