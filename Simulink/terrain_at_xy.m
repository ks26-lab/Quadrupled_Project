function [z, mu, hit_id] = terrain_at_xy(engine,x,y)
%TERRAIN_AT_XY Single geometry query used by collision and rendering checks.
% The patch modifies friction, not support elevation.
c = engine.conveyor;
z = c.height; mu = c.friction_mu; hit_id = 0;
if abs(x-c.center(1)) > c.length/2 || abs(y-c.center(2)) > c.width/2
    z = 0; mu = 0; return; % Outside belt: floor at z=0.
end
for i = 1:numel(engine.obstacles)
    o = engine.obstacles(i);
    if ~o.active, continue; end
    if abs(x-o.pos(1)) > o.size(1)/2 || abs(y-o.pos(2)) > o.size(2)/2
        continue;
    end
    if strcmp(o.type,'patch')
        if isfinite(o.friction_mu), mu = o.friction_mu; end
        continue;
    end
    base_z = o.pos(3)-o.size(3)/2;
    if strcmp(o.type,'ramp')
        p = min(1,max(0,(x-(o.pos(1)-o.size(1)/2))/o.size(1)));
        top_z = base_z+o.size(3)*p;
    else
        top_z = base_z+o.size(3);
    end
    if top_z > z, z=top_z; hit_id=o.id; end
end
end
