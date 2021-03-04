function pL = plotShip(t,posx,posy,psi,obs_ori,ship_size,no_ship,color,alpha,Ecolor,EWidth)

T   = linspace(t(1), t(end), no_ship);
X   = interp1(t, posx, T);
Y   = interp1(t, posy, T);
PSI = interp1(t, deg2rad(psi), T);

%%
% figure
% plot(X(end),Y(end),'b*','MarkerSize',10);
plot(posx,posy,'k'); hold on;
axis equal; grid on;

%% Ship Dimension
if ship_size == 3
    LH  = 3/2;     % Half of length
    BH  = 0.5/2;   % Half of breadth
    Fwd = 1;       % Mid to Bow Chine length
else
    LH  = 178/2;   % Half of length
    BH  = 32/2;    % Half of breadth
    Fwd = 50;      % Mid to Bow Chine length
end

X2 = [LH, Fwd, -LH, -LH, Fwd, LH];
Y2 = [0, BH, BH, -BH, -BH, 0];


for i = 1:1:length(T)

    Rot_ship = [cos(PSI(i)) -sin(PSI(i)); sin(PSI(i)) cos(PSI(i))];
    XYR = Rot_ship*[X2; Y2];
    
    Ship = polyshape(XYR(1,:)+X(i),XYR(2,:)+Y(i));
    pL = plot(Ship,'FaceColor',color,'LineWidth',EWidth,'FaceAlpha',alpha,'EdgeColor',Ecolor); hold on;
    

end

hold off; axis equal;

end
