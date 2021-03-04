function pL = plotAShip(X, Y, PSI, color, alpha, Ecolor, EWidth)
%% Ship Dimension

LH  = 3/2;     % Half of length
BH  = 0.5/2;   % Half of breadth
Fwd = 1;       % Mid to Bow Chine length


X2 = [LH, Fwd, -LH, -LH, Fwd, LH];
Y2 = [0, BH, BH, -BH, -BH, 0];

Rot_ship = [cos(PSI) -sin(PSI); sin(PSI) cos(PSI)];
XYR = Rot_ship*[X2; Y2];

Ship = polyshape(XYR(1,:)+X,XYR(2,:)+Y);
pL = plot(Ship,'FaceColor',color,'LineWidth',EWidth,'FaceAlpha',alpha,'EdgeColor',Ecolor);

axis equal;

end