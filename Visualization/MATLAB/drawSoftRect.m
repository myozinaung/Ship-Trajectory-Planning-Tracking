function drawSoftRect(sRect)
[sRectNo,~] = size(sRect);

for i = 1:sRectNo
    
    x0   = sRect(i,1);
    y0   = sRect(i,2);
    a    = sRect(i,3);
    b    = sRect(i,4);
    psi  = sRect(i,5);
    x_exponent = sRect(i,7); % The exponent in the equation ( "2" for ellipse)
    y_exponent = sRect(i,8);

    th = 0:pi/50:2*pi-pi/50;
    Rot = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    X = zeros(1,length(th));
    for it = 1:length(th)
        if cos(th(it)) < 0
            X(it) = -a*abs((cos(th(it)))^(1/(x_exponent/2))); % Stern
            
        else
            X(it) =  a*abs((cos(th(it)))^(1/(x_exponent/2))); % Bow, use "2" in place of 4 if elliple-like bow is needed
        end
    end
    Y = zeros(1,length(th));
    for it = 1:length(th)
        if sin(th(it)) < 0
            Y(it) = -b*abs((sin(th(it)))^(1/(y_exponent/2))); % Stern
            
        else
            Y(it) =  b*abs((sin(th(it)))^(1/(y_exponent/2))); % Bow, use "2" in place of 4 if elliple-like bow is needed
        end
    end
    
%     Y = b*sin(th); % assuming y_exponent is 2
    
    SRectR = Rot*[X; Y];
    
    SRect = polyshape(SRectR(1,:)+x0,SRectR(2,:)+y0);
    plot(SRect,'LineStyle',"-",'LineWidth',0.5,'FaceColor',[0 1 1],'FaceAlpha',0.2,'DisplayName','SoftRectangle');
    hold on;

end

end