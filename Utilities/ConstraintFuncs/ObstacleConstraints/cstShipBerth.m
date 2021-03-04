function [c, ceq] = cstShipBerth(t,x,u,p)
xpos0 = x(1,:);
ypos0 = x(2,:);
psiS0 = x(3,:);
c = [];

%% Obstacles
if isfield(p.obs,'rect')
    rect = p.obs.rect;
    [rectNo,~] = size(rect);
end

if isfield(p.obs,'circ')
    circ = p.obs.circ;
    [circNo,~] = size(circ);
end

if isfield(p.obs,'elli')
    elli = p.obs.elli;
    [elliNo,~] = size(elli);
end

if isfield(p.obs,'sRect')
    sRect = p.obs.sRect;
    [sRectNo,~] = size(sRect);
end

if isfield(p.obs,'rect') || isfield(p.obs,'circ') || isfield(p.obs,'elli') || isfield(p.obs,'sRect')
    % 6 points for 6 series of circles for Ego Ship
    point = -1.25:0.5:1.25; % for 3m Esso Osaka
%     point = 0; % for 3m Esso Osaka, only CG
%     point = -80:32:80; % for large ship
    for i = 1:length(point)
        xposP = xpos0 + point(i).*cos(psiS0);
        yposP = ypos0 + point(i).*sin(psiS0);

        if isfield(p.obs,'rect')
            for ir = 1:rectNo
                rectCst = cstRectangle(xposP,yposP,rect(ir,:));
                c = [c, rectCst];
            end
        end

        if isfield(p.obs,'circ')
            for ic = 1:circNo
                circCst = cstCircle(xposP,yposP,circ(ic,:));
                c = [c, circCst];
            end
        end
        
        if isfield(p.obs,'elli')
            for ie = 1:elliNo
                elliCst = cstEllipse(xposP,yposP,elli(ie,:),t);
                c = [c, elliCst];
            end
        end
        
        if isfield(p.obs,'sRect')
            for is = 1:sRectNo
                sRectCst = cstSoftRect(xposP,yposP,sRect(is,:),t);
                c = [c, sRectCst];
            end
        end        
        
    end
end

%% For XY Bounds % if XY Bound are inf/-inf this will result fmincon error
% xposB = xpos0 + 1.5*cos(psiS0); % Bow
% % yposB = ypos0 + 1.25*sin(psiS0);
% 
% xposS = xpos0 - 1.5*cos(psiS0); % Stern
% % yposS = ypos0 - 1.25*sin(psiS0);
% % 
% % % For Bound Constraints (e.g. Berth), only bow point and stern points are needed
% xMinB = -(xposB - p.bndXY(1));
% xMinS = -(xposS - p.bndXY(1));
% c = [c, xMinB, xMinS];
% % 

% xMaxB =  xposB - p.bndXY(2);
% xMaxS =  xposS - p.bndXY(2);
% c = [c, xMaxB, xMaxS];
% 
% % yMinB = -(yposB - p.bndXY(3));
% % yMinS = -(yposS - p.bndXY(3));
% % c = [c, yMinB, yMinS];
% 
% % yMaxB = yposB - p.bndXY(4);
% % yMaxS = yposS - p.bndXY(4);
% % c = [c, yMaxB, yMaxS];

%% For control rate limits (in case of 6 sates Model)
% deltaRateBound = deg2rad(40); % deg/s
% rpsRateBound   = 5; % rps/s
% 
% cstDeltaR = abs((u(1,2:end) - u(1,1:end-1))./(t(2:end) - t(1:end-1))) - deltaRateBound;
% cstRpsR   = abs((u(2,2:end) - u(2,1:end-1))./(t(2:end) - t(1:end-1))) - rpsRateBound;
% c = [c, cstDeltaR, cstRpsR];

%% Limit the zero-crossing using pentalty for near-zero control usage
% deltaMax = deg2rad(35);
% dletaZeroMax = 100;
% cstDeltaZero = (1*(-(x(7,:)./deltaMax).^10 + 1)*t(end)) - dletaZeroMax;
% c = [c, cstDeltaZero];
% 
% rpsMax = 15;
% rpsZeroMax = 200;
% cstRPSZero = (1*(-(x(8,:)./rpsMax).^10 + 1)*t(end)) - rpsZeroMax;
% c = [c, cstRPSZero];

%% Minimize Gear Chnages
% countGear = 0;
% for i = 1:length(t)-1
%     if u(2,i)*u(2,i+1) < 0
%         countGear = countGear + 1;
%     end
% end
% cstGear = countGear - 100;
% c = [c, cstGear];


%% Collect all constraits
ceq = [];

%%
% cstMaxRPShalf = u(2,1:length(t)/2) - 15;
% ceq = [ceq, cstMaxRPShalf];
% 
% cstMinRPShalf = u(2,length(t)/2+5:end-1) + 15;
% ceq = [ceq, cstMinRPShalf];
end