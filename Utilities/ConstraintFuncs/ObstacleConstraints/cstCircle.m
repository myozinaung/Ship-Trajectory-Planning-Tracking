function cst = cstCircle(x,y,circ)

x0  = circ(1);
y0  = circ(2);
rad = circ(3);

% cst = -((x-x0).^2 + (y-y0).^2 - rad^2); % Original circle

cst = -(log((x-x0).^2 + (y-y0).^2) - log(rad^2)); % Faster NLP when using log on both side
% natural log is faster than log10

% epsilon = 1e-5;
% cst = -(log((x-x0).^2 + (y-y0).^2 + epsilon) - log(rad^2 + epsilon)); % a small number is added to both side
% % Not much difference, without epsilon seems to be better
end