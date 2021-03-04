figure;
xposWP = (xWP_low(1,:)+xWP_upp(1,:))./2;
yposWP = (xWP_low(2,:)+xWP_upp(2,:))./2;
psiWP  = (xWP_low(3,:)+xWP_upp(3,:))./2;
for i = 1:size(xWP_low,2)
    plotAShip(xposWP(i), yposWP(i), psiWP(i), 'red', 0.1, 'red', 1); hold on;
    plotCircleSeries(xposWP(i), yposWP(i), psiWP(i), radi_inflate)
end
plot(xposWP, yposWP, 'o-'); 
hold on; grid on; axis equal;

for i = 1:length(obs_ori)
    if isfield(obs_ori{i},'circ')
        drawCircle(obs_ori{i}.circ); hold on;
    end
    if isfield(obs_ori{i},'rect')
        drawRect(obs_ori{i}.rect); hold on;
    end
    if isfield(obs_ori{i},'elli')
        drawEllipse(obs_ori{i}.elli); hold on;
    end
    if isfield(obs_ori{i},'sRect')
        drawSoftRect(obs_ori{i}.sRect); hold on; 
    end
    
%     if i == 1
%         break;
%     end
end
