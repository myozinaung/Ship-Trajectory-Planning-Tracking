function drawCircle(circ)
[circNo,~] = size(circ);

for i = 1:circNo
    
    x0 = circ(i,1);
    y0 = circ(i,2);
    r  = circ(i,3);

    th = 0:pi/50:2*pi-pi/50;
    X  = r*cos(th) + x0;
    Y  = r*sin(th) + y0;
    Cir = polyshape(X,Y);
    plot(Cir,'LineStyle',"--",'LineWidth',0.5,'FaceColor',[0 0 0],'FaceAlpha',0.1,'DisplayName','Circle');
    hold on;

end

end