function app = create_3d_simulation_app(cfg)
%CREATE_3D_SIMULATION_APP 3D view and runtime telemetry for reduced-order engine.
% Visuals and numeric displays read from simEngine, not independent constants.
if nargin<1, cfg=struct(); end
engine=init_3d_simulation(cfg);
fig=figure('Name','Quadruped | Conveyor Simulation (Reduced-order)', ...
    'Color',[.12 .12 .14],'Position',[40 40 1440 850], ...
    'MenuBar','none','NumberTitle','off','CloseRequestFcn',@onClose);
ax=axes('Parent',fig,'Position',[.035 .34 .63 .61], ...
    'Color',[.15 .15 .18],'XColor',[.8 .8 .8], ...
    'YColor',[.8 .8 .8],'ZColor',[.8 .8 .8]);
hold(ax,'on');grid(ax,'on');axis(ax,'equal');view(ax,40,25);
xlabel(ax,'X (m)');ylabel(ax,'Y (m)');zlabel(ax,'Z (m)');
title(ax,'Conveyor, solid obstacles and robot','Color',[.9 .9 .9]);
ap=axes('Parent',fig,'Position',[.72 .68 .25 .25], ...
    'Color',[.15 .15 .18],'XColor',[.8 .8 .8],'YColor',[.8 .8 .8]);
hold(ap,'on');grid(ap,'on');ylabel(ap,'Pitch (deg)');title(ap,'Measured pitch and reference');
pitch_line=plot(ap,nan,nan,'c-','LineWidth',1.5);
ref_line=plot(ap,nan,nan,'r--','LineWidth',1.1);
at=axes('Parent',fig,'Position',[.72 .36 .25 .25], ...
    'Color',[.15 .15 .18],'XColor',[.8 .8 .8],'YColor',[.8 .8 .8]);
hold(at,'on');grid(at,'on');xlabel(at,'Time (s)');ylabel(at,'Torque (N m)');
title(at,'Actual controller output');
torque_line=plot(at,nan,nan,'m-','LineWidth',1.5);
limit_top=yline(at,engine.robot.u_sat,'k:');
limit_bottom=yline(at,-engine.robot.u_sat,'k:');
panel=uipanel('Parent',fig,'Position',[.03 .02 .94 .29], ...
    'Title','CONTROLS | Values shown are from engine state', ...
    'BackgroundColor',[.18 .18 .22],'ForegroundColor','white');
btn=uicontrol(panel,'Style','pushbutton','String','PAUSE', ...
    'Position',[15 130 80 32],'Callback',@togglePause);
uicontrol(panel,'Style','pushbutton','String','RESET', ...
    'Position',[105 130 80 32],'Callback',@resetSimulation);
uicontrol(panel,'Style','text','String','Belt (m/s)', ...
    'Position',[200 140 105 20]);
speed=uicontrol(panel,'Style','slider','Min',-.30,'Max',.30, ...
    'Position',[310 140 130 20],'Callback',@changeSpeed);
speed_label=uicontrol(panel,'Style','text','Position',[450 140 85 20]);
uicontrol(panel,'Style','text','String','Cadence (Hz)', ...
    'Position',[200 100 105 20]);
freq=uicontrol(panel,'Style','slider','Min',.5,'Max',2, ...
    'Position',[310 100 130 20],'Callback',@changeFreq);
freq_label=uicontrol(panel,'Style','text','Position',[450 100 85 20]);
uicontrol(panel,'Style','text','String','Step height (m)', ...
    'Position',[200 60 105 20]);
stepht=uicontrol(panel,'Style','slider','Min',.02,'Max',.08, ...
    'Position',[310 60 130 20],'Callback',@changeHeight);
height_label=uicontrol(panel,'Style','text','Position',[450 60 85 20]);
scenario=uicontrol(panel,'Style','popupmenu', ...
    'String',{'All obstacles (default)','Moving conveyor','Obstacle traversal', ...
    'Ramp and friction patch','Torque disturbance','RF contact loss'}, ...
    'Position',[560 140 320 24],'Callback',@selectScenario);
uicontrol(panel,'Style','pushbutton','String','INJECT TORQUE', ...
    'Position',[560 90 150 32],'Callback',@injectTorque);
uicontrol(panel,'Style','pushbutton','String','TOGGLE RF FAULT', ...
    'Position',[725 90 155 32],'Callback',@toggleFault);
status=uicontrol(panel,'Style','text','Position',[15 12 1040 30], ...
    'HorizontalAlignment','left','String','');
% All scene geometry is derived from current engine state.
scene=struct();
rebuildScene();
body=patch(ax,'XData',zeros(1,4),'YData',zeros(1,4), ...
    'ZData',zeros(1,4),'FaceColor',[.25 .45 .85], ...
    'EdgeColor',[.1 .8 .9],'LineWidth',1.5);
imu=plot3(ax,0,0,0,'yo','MarkerFaceColor','y');
legs=gobjects(4,1); feet=gobjects(4,1);
colors={[.2 .8 .9],[.9 .4 .2],[.2 .8 .5],[.9 .8 .2]};
for k=1:4
    legs(k)=plot3(ax,zeros(1,3),zeros(1,3),zeros(1,3), ...
        'Color',colors{k},'LineWidth',3);
    feet(k)=plot3(ax,0,0,0,'o','MarkerSize',7);
end
updateControls();
updateView();
period=1/engine.physics.visual_fps;
steps_per_tick=max(1,round(period/engine.time.dt));
sim_timer=timer('ExecutionMode','fixedSpacing','Period',period, ...
    'TimerFcn',@tick,'ErrorFcn',@timerError);
start(sim_timer);
app=struct('fig',fig,'timer',sim_timer, ...
    'getEngine',@() engine,'reset',@resetSimulation);

    function rebuildScene()
        if isfield(scene,'objects')
            for j=1:numel(scene.objects)
                if isgraphics(scene.objects(j)),delete(scene.objects(j));end
            end
        end
        objects=gobjects(0);
        c=engine.conveyor;
        xmin=c.center(1)-c.length/2; xmax=c.center(1)+c.length/2;
        ymin=c.center(2)-c.width/2; ymax=c.center(2)+c.width/2;
        objects(end+1)=patch(ax,[xmin xmax xmax xmin], ...
            [ymin ymin ymax ymax],zeros(1,4),[.2 .2 .24], ...
            'EdgeColor',[.35 .35 .4]);
        objects(end+1)=patch(ax,[xmin xmax xmax xmin], ...
            [ymin ymin ymax ymax],c.height*ones(1,4),[.12 .12 .15], ...
            'EdgeColor',[.45 .45 .55]);
        n=max(1,ceil(c.length/c.stripe_pitch));
        stripes=gobjects(n,1);
        for j=1:n
            stripes(j)=patch(ax,zeros(1,4),zeros(1,4), ...
                (c.height+.001)*ones(1,4),[.85 .72 .10], ...
                'EdgeColor','none','FaceAlpha',.75);
        end
        objects=[objects stripes(:)'];
        for j=1:numel(engine.obstacles)
            o=engine.obstacles(j);
            if ~o.active,continue;end
            objects(end+1)=drawObstacle(o);
        end
        scene.objects=objects; scene.stripes=stripes;
        scene.xmin=xmin; scene.xmax=xmax;
        scene.ymin=ymin; scene.ymax=ymax;
        xlim(ax,[xmin-.1 xmax+.1]);ylim(ax,[ymin-.3 ymax+.3]);
        zlim(ax,[0 max(.75,c.height+engine.robot.standing_height+.3)]);
    end

    function h=drawObstacle(o)
        x=o.pos(1);y=o.pos(2);z=o.pos(3);
        a=o.size(1)/2;b=o.size(2)/2;d=o.size(3)/2;
        if strcmp(o.type,'patch')
            ztop=engine.conveyor.height+.002;
            h=patch(ax,[x-a x+a x+a x-a],[y-b y-b y+b y+b], ...
                ztop*ones(1,4),o.color,'EdgeColor','w', ...
                'LineWidth',1.3,'FaceAlpha',.85);
            return;
        end
        z0=z-d;z1=z+d;
        V=[x-a y-b z0; x+a y-b z0; x+a y+b z0; x-a y+b z0; ...
           x-a y-b z1; x+a y-b z1; x+a y+b z1; x-a y+b z1];
        if strcmp(o.type,'ramp')
            V([5 8],3)=z0; % Wedge rises along +X; same shape as terrain query.
        end
        F=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
        h=patch(ax,'Vertices',V,'Faces',F,'FaceColor',o.color, ...
            'EdgeColor',[.03 .03 .03],'LineWidth',1.2,'FaceAlpha',.98);
    end

    function tick(~,~)
        if ~isgraphics(fig),return;end
        if ~engine.time.is_paused
            for j=1:steps_per_tick
                engine=step_3d_simulation(engine);
            end
        end
        updateView();
    end

    function updateView()
        s=engine.state;r=engine.robot;
        a=s.rpy(2);R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];
        L=r.body_length/2;W=r.body_width/2;H=r.body_height/2;
        verts= s.pos+R*[L -L -L L;W W -W -W;H H H H];
        set(body,'XData',verts(1,:),'YData',verts(2,:), ...
            'ZData',verts(3,:));
        marker=s.pos+R*[0;0;H+.01];
        set(imu,'XData',marker(1),'YData',marker(2),'ZData',marker(3));
        for j=1:4
            hip=s.pos+R*r.hip_positions(j,:)';
            foot=s.foot_pos_world(j,:)';
            knee=(hip+foot)/2+R*[.03;0;.02];
            set(legs(j),'XData',[hip(1) knee(1) foot(1)], ...
                'YData',[hip(2) knee(2) foot(2)], ...
                'ZData',[hip(3) knee(3) foot(3)]);
            if s.contact_states(j), col=[.2 .9 .3];else,col=[.9 .2 .2];end
            set(feet(j),'XData',foot(1),'YData',foot(2),'ZData',foot(3), ...
                'MarkerFaceColor',col,'MarkerEdgeColor','k');
        end
        c=engine.conveyor;
        for j=1:numel(scene.stripes)
            x=scene.xmin+mod((j-1)*c.stripe_pitch-c.texture_offset,c.length);
            x2=min(x+.05,scene.xmax);
            set(scene.stripes(j),'XData',[x x2 x2 x], ...
                'YData',[scene.ymin+.05 scene.ymin+.05 scene.ymax-.05 scene.ymax-.05]);
        end
        h=engine.history;
        if ~isempty(h.time)
            idx=max(1,numel(h.time)-399):numel(h.time);
            ts=h.time(idx);
            set(pitch_line,'XData',ts,'YData',rad2deg(h.theta(idx)));
            set(ref_line,'XData',ts,'YData',rad2deg(h.theta_ref(idx)));
            set(torque_line,'XData',ts,'YData',h.u(idx));
            xlim(ap,[ts(1) max(ts(1)+1,ts(end))]);
            xlim(at,[ts(1) max(ts(1)+1,ts(end))]);
        end
        set(limit_top,'Value',r.u_sat);set(limit_bottom,'Value',-r.u_sat);
        set(status,'String',sprintf(['t=%.2f s | contacts=%d/4 | slip=%.0f%% | ' ...
            'pitch=%.2f deg | torque=%.3f N m | disturbance=%.3f N m | ' ...
            'IK feasible=%d/4'],engine.time.current,nnz(s.contact_states), ...
            100*s.slip_ratio,rad2deg(s.rpy(2)),s.u_pitch, ...
            s.tau_external,nnz(s.ik_feasible)));
        drawnow limitrate;
    end

    function updateControls()
        c=engine.conveyor;g=engine.gait;
        v=c.speed*c.direction;
        set(speed,'Value',max(speed.Min,min(speed.Max,v)));
        set(freq,'Value',max(freq.Min,min(freq.Max,g.frequency)));
        set(stepht,'Value',max(stepht.Min,min(stepht.Max,g.step_height)));
        set(speed_label,'String',sprintf('%.2f m/s',v));
        set(freq_label,'String',sprintf('%.2f Hz',g.frequency));
        set(height_label,'String',sprintf('%.3f m',g.step_height));
        if engine.time.is_paused
            set(btn,'String','RESUME');
        else
            set(btn,'String','PAUSE');
        end
    end

    function togglePause(~,~)
        engine.time.is_paused=~engine.time.is_paused;
        updateControls();
    end
    function resetSimulation(~,~)
        engine=init_3d_simulation(cfg);
        set(scenario,'Value',1);
        rebuildScene();updateControls();updateView();
    end
    function changeSpeed(src,~)
        v=src.Value;engine.conveyor.speed=abs(v);
        if v==0
            engine.conveyor.direction=1;
        else
            engine.conveyor.direction=sign(v);
        end
        updateControls();
    end
    function changeFreq(src,~)
        engine.gait.frequency=src.Value;updateControls();
    end
    function changeHeight(src,~)
        engine.gait.step_height=src.Value;updateControls();
    end
    function injectTorque(~,~)
        engine.state.external_torque=engine.physics.disturbance_torque;
        engine.state.external_torque_until=engine.time.current+ ...
            engine.physics.disturbance_duration;
    end
    function toggleFault(~,~)
        if engine.state.contact_loss_leg==2
            engine.state.contact_loss_leg=0;
        else
            engine.state.contact_loss_leg=2;
        end
    end
    function selectScenario(src,~)
        engine=init_3d_simulation(cfg);
        for j=1:numel(engine.obstacles),engine.obstacles(j).active=false;end
        switch src.Value
            case 1
                for j=1:numel(engine.obstacles)
                    engine.obstacles(j).active=true;
                end
            case 2,engine.conveyor.speed=.12;
            case 3
                engine.obstacles(1).active=true;
                engine.obstacles(3).active=true;
                engine.conveyor.speed=.08;
            case 4
                engine.obstacles(2).active=true;
                engine.obstacles(4).active=true;
                engine.conveyor.speed=.06;
            case 5
                engine.conveyor.speed=.10;injectTorque([],[]);
            case 6
                engine.state.contact_loss_leg=2;
        end
        rebuildScene();updateControls();updateView();
    end
    function timerError(~,event)
        warning('3D simulation timer failed: %s',event.Data.Message);
        if isgraphics(fig),set(status,'String',event.Data.Message);end
    end
    function onClose(~,~)
        if exist('sim_timer','var') && isvalid(sim_timer)
            stop(sim_timer);delete(sim_timer);
        end
        if isgraphics(fig),delete(fig);end
    end
end
