function obs = inflateObs(obs,radInfl)

if isfield(obs,'rect')
    obs.rect(:,3:4) = obs.rect(:,3:4) + 2*radInfl;
end

if isfield(obs,'circ') 
    obs.circ(:,3)   = obs.circ(:,3) + radInfl;
end

if isfield(obs,'elli') 
    obs.elli(:,3:4) = obs.elli(:,3:4) + radInfl;
end

if isfield(obs,'sRect') 
    obs.sRect(:,3:4) = obs.sRect(:,3:4) + radInfl;
end