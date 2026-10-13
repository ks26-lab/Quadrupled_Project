function engine = step_3d_simulation(engine)
%STEP_3D_SIMULATION One deterministic reduced-order simulation step.
% This is NOT a rigid-body dynamics solver: vertical pose is terrain-following,
% and contact force is a spring-damper diagnostic, not a vertical force balance.
if engine.time.is_paused, return; end
dt = engine.time.dt; t = engine.time.current;
r = engine.robot; c = engine.conveyor; g = engine.gait;
s = engine.state; ph = engine.physics;
if c.running, vb = c.direction*c.speed; else, vb = 0; end
c.texture_offset = mod(c.texture_offset+vb*dt,c.stripe_pitch);
phase = mod(t*max(g.frequency,0.01)+g.phase_offsets(:),1);
stance_command = phase >= 0.5;
err = s.theta_reference-s.rpy(2);
% Original adaptation thresholds retained; these are heuristic controls.
if abs(err)>0.15
    ht=g.step_height*1.25; len=g.step_length*0.70;
elseif abs(err)>0.05
    ht=g.step_height*1.10; len=g.step_length*0.85;
else
    ht=g.step_height; len=g.step_length;
end
candidate = s.integrator+err*dt;
unsat = r.Kp*err+r.Ki*candidate;
u = min(r.u_sat,max(-r.u_sat,unsat));
if ~((unsat>r.u_sat && err>0)||(unsat< -r.u_sat && err<0))
    s.integrator=candidate;
end
% Foot target coordinates use the existing project's nominal-foot convention.
% Verify geom.nominalFootPosition frame against geom.hipPosition separately.
foot_local = zeros(4,3);
for k=1:4
    p=phase(k);
    if p<0.5
        a=p/0.5;
        dx=len*(a-sin(2*pi*a)/(2*pi))-len/2;
        dz=ht*(1-cos(2*pi*a))/2;
    else
        a=(p-0.5)/0.5; dx=len/2-len*a; dz=0;
    end
    foot_local(k,:)=r.nominal_feet(k,:)+[dx 0 dz];
end
% IK is based on the same foot targets as the renderer; feasibility logged.
q=zeros(12,1); feasible=true(4,1);
for k=1:4
    xyz=foot_local(k,:); x=xyz(1); y=xyz(2); z=xyz(3);
    q1=atan2(y,-z); rr=hypot(y,z); D=hypot(x,rr);
    cs=(D^2-r.L1^2-r.L2^2)/(2*r.L1*r.L2);
    if abs(cs)>1
        feasible(k)=false; continue;
    end
    q3=acos(min(1,max(-1,cs)));
    q2=atan2(x,rr)-atan2(r.L2*sin(q3),r.L1+r.L2*cos(q3));
    q((k-1)*3+(1:3))=[q1;q2;q3];
end
dq=zeros(12,1);
dq([2 5])=r.k_torque_to_q*u;
dq([8 11])=-r.k_torque_to_q*u;
s.q_cmd=q+dq;
% Predict feet from current pose for terrain queries.
[foot,z_ground,mu] = foot_and_terrain(engine,s,foot_local);
% Terrain-following vertical kinematics (not a force-integrated vertical axis).
% Uses stance legs only; no vertical motion is inferred from swing legs.
valid=stance_command;
if s.contact_loss_leg>=1 && s.contact_loss_leg<=4
    valid(s.contact_loss_leg)=false;
end
if any(valid)
    z_target=c.height+r.standing_height+mean(z_ground(valid)-c.height);
else
    z_target=s.pos(3);
end
% Recompute feet after vertical update, to keep rendering and contact consistent.
s.vel(3)=(z_target-s.pos(3))/dt;
s.pos(3)=z_target;
[foot,z_ground,mu]=foot_and_terrain(engine,s,foot_local);
normal=zeros(4,1); contacts=false(4,1);
for k=1:4
    if s.contact_loss_leg==k, continue; end
    penetration=z_ground(k)-foot(k,3);
    % Contact only when near surface; never manufacture a baseline 50 N.
    contacts(k)=stance_command(k) && penetration >= -ph.contact_tolerance;
    prevz=s.foot_pos_world(k,3);
    if t==0, prevz=foot(k,3); end
    pen_rate=(prevz-foot(k,3))/dt;
    if contacts(k)
        normal(k)=max(0,ph.contact_stiffness*max(0,penetration) + ...
            ph.contact_damping*max(0,pen_rate));
    end
end
% Friction-dependent conveyor coupling is a documented reduced-order model.
% It is not a Coulomb contact-force or momentum solver.
if any(contacts)
    mu_effective=mean(mu(contacts));
    traction=min(1,max(0,mu_effective/max(ph.slip_mu_scale,eps)));
    world_v=g.vx_cmd-vb*traction;
    s.slip_ratio=1-traction;
else
    world_v=0; s.slip_ratio=1;
end
s.vel(1)=world_v;
s.pos(1)=s.pos(1)+world_v*dt;
% Terrain gradient affects pitch through a configurable approximate moment.
front=z_ground(1:2); rear=z_ground(3:4);
tau_terrain=-ph.terrain_pitch_gain*(mean(front)-mean(rear));
if t < s.external_torque_until
    tau_ext=s.external_torque;
else
    tau_ext=0;
    s.external_torque=0;
end
acc=(u+tau_terrain+tau_ext-r.B_pitch*s.omega(2))/r.J_pitch;
s.omega(2)=s.omega(2)+acc*dt;
s.rpy(2)=s.rpy(2)+s.omega(2)*dt;
% Recompute world coordinates at final body pose (removes one-step visual lag).
[foot,z_ground,mu]=foot_and_terrain(engine,s,foot_local);
s.foot_pos_world=foot;
s.contact_states=contacts;
s.contact_force=normal;
s.contact_mu=mu;
s.terrain_z=z_ground;
s.ik_feasible=feasible;
s.u_pitch=u;
s.tau_terrain=tau_terrain;
s.tau_external=tau_ext;
engine.state=s; engine.conveyor=c;
engine.time.current=t+dt;
h=engine.history;
h.time(end+1)=engine.time.current;
h.theta(end+1)=s.rpy(2);
h.theta_ref(end+1)=s.theta_reference;
h.u(end+1)=u;
h.q(:,end+1)=s.q_cmd;
h.conveyor_x(end+1)=c.texture_offset;
h.contact_force(:,end+1)=normal;
h.contact_count(end+1)=nnz(contacts);
h.slip_ratio(end+1)=s.slip_ratio;
h.tau_external(end+1)=tau_ext;
h.terrain_z(:,end+1)=z_ground;
engine.history=h;
end

function [foot,terrain,mu] = foot_and_terrain(engine,s,foot_local)
r=engine.robot; a=s.rpy(2);
R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];
foot=zeros(4,3); terrain=zeros(4,1); mu=zeros(4,1);
for k=1:4
    xyz=s.pos+R*(r.hip_positions(k,:)'+foot_local(k,:)');
    foot(k,:)=xyz';
    [terrain(k),mu(k)]=terrain_at_xy(engine,xyz(1),xyz(2));
end
end
