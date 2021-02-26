function drawRect(rect)
[rectNo,~] = size(rect);

for i = 1:rectNo
    x0  = rect(i,1);
    y0  = rect(i,2);
    a   = rect(i,3);
    b   = rect(i,4);
    psi = rect(i,5);

    X = [-a/2 a/2 a/2 -a/2 -a/2];
    Y = [b/2 b/2 -b/2 -b/2 b/2];

    Rot = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    P = Rot*[X;Y];
    Rect = polyshape(P(1,:)+x0,P(2,:)+y0);
    plot(Rect,'LineStyle',"--",'LineWidth',0.5,'FaceColor',[0 0 0],'FaceAlpha',0.1,'DisplayName','Rectangle');
    hold on;
end