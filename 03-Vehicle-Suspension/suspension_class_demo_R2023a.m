function suspension_class_demo_R2023a
% SUSPENSION_CLASS_DEMO_R2023A  One-DOF suspension for MATLAB R2023a.
%
% Requirements: MATLAB R2023a + Simulink + Simscape.
% Running this function creates suspension_1dof_R2023a.slx, simulates a
% single rounded road bump, and displays both a plot and a 2-D animation.
%
% MODEL (displacements are measured from static equilibrium):
%    m*x'' + c*(x' - r') + k*(x-r) = 0
%
% Classroom experiments:
%   c = 150:   worn / weak shock absorber
%   c = 1200:  baseline shock absorber
%   c = 5000:  very strong damping
%   k = 30000: stiff spring (compare against 15000)
%
% Change the numbers below, then run suspension_class_demo_R2023a again.

%% 1. Parameters students can change
m = 250;          % kg, supported car-body mass
k = 15000;        % N/m, suspension spring stiffness
c = 3872;         % N*s/m, viscous damping coefficient
bumpHeight = 0.10;% m (10 cm)
bumpStart = 1.0;  % s, road bump begins
bumpTime = 0.50;  % s, time needed to cross bump
simTime = 6.0;    % s, total simulation time
showAnimation = true;

%% 2. Road shape and its velocity (velocity drives the physical base)
dt = 0.002;
t = (0:dt:simTime).';
r = zeros(size(t));
vr = zeros(size(t));
inside = (t >= bumpStart) & (t <= bumpStart+bumpTime);
tau = t(inside)-bumpStart;
r(inside)  = bumpHeight/2 * (1-cos(2*pi*tau/bumpTime));
vr(inside) = bumpHeight*pi/bumpTime * sin(2*pi*tau/bumpTime);
% This is a single smooth mound: the road returns to its starting height.
assignin('base','susp_road_m',timeseries(r,t));
assignin('base','susp_road_velocity_mps',timeseries(vr,t));

%% 3. Automatically create a physical Simscape model
if ~license('test','Simulink') || isempty(ver('simscape'))
    error('Simulink and Simscape must be installed and licensed.');
end
model = 'suspension_1dof_R2023a';
if bdIsLoaded(model)
    close_system(model,0); % Discard prior unsaved changes to generated model
end
new_system(model);
open_system(model);
load_system('fl_lib');
load_system('nesl_utility');

% Named paths: names also serve as labels on the student-facing diagram.
mass  = [model '/Mass'];
spring= [model '/Spring'];
damper= [model '/Damper'];
source= [model '/Road_velocity_source'];
sensor= [model '/Body_motion_sensor'];
ref   = [model '/Mechanical_reference'];
solver= [model '/Solver_configuration'];
inVel = [model '/Road_velocity_input'];
psIn  = [model '/Simulink_to_physical'];
psOut = [model '/Physical_to_Simulink'];
inRoad= [model '/Road_height_plot'];
mux   = [model '/Compare_body_and_road'];
scope = [model '/Scope'];
toWks = [model '/Body_displacement_data'];

% Simscape mechanical components
add_block('fl_lib/Mechanical/Translational Elements/Mass',mass, ...
    'Position',[400 75 450 125],'mass',num2str(m));
add_block('fl_lib/Mechanical/Translational Elements/Translational Spring',spring, ...
    'Position',[405 205 455 255],'spr_rate',num2str(k));
add_block('fl_lib/Mechanical/Translational Elements/Translational Damper',damper, ...
    'Position',[590 205 640 255],'D',num2str(c));
add_block('fl_lib/Mechanical/Mechanical Sources/Ideal Translational Velocity Source',source, ...
    'Position',[500 370 560 430]);
add_block('fl_lib/Mechanical/Mechanical Sensors/Ideal Translational Motion Sensor',sensor, ...
    'Position',[655 70 725 135], ...
    'position_port','true','velocity_port','false');
add_block('fl_lib/Mechanical/Translational Elements/Mechanical Translational Reference',ref, ...
    'Position',[725 390 765 430]);
add_block('nesl_utility/Solver Configuration',solver, ...
    'Position',[365 400 415 440]);

% Simulink signal sources and physical/signal converters
add_block('simulink/Sources/From Workspace',inVel, ...
    'Position',[45 375 200 410], ...
    'VariableName','susp_road_velocity_mps','Interpolate','on');
add_block(sprintf('nesl_utility/Simulink-PS\nConverter'),psIn, ...
    'Position',[265 370 335 415],'Unit','m/s');
add_block(sprintf('nesl_utility/PS-Simulink\nConverter'),psOut, ...
    'Position',[795 80 870 125],'Unit','m');
add_block('simulink/Sources/From Workspace',inRoad, ...
    'Position',[765 255 915 290], ...
    'VariableName','susp_road_m','Interpolate','on');
add_block('simulink/Signal Routing/Mux',mux, ...
    'Position',[970 113 975 183],'Inputs','2');
add_block('simulink/Sinks/Scope',scope, ...
    'Position',[1060 125 1120 170]);
add_block('simulink/Sinks/To Workspace',toWks, ...
    'Position',[985 40 1120 75], ...
    'VariableName','susp_body_x','SaveFormat','Timeseries');

%% 4. Connect conserving mechanical ports (not ordinary signal wires)
% Body node: mass, spring, damper and absolute motion sensor.
connect_physical(mass,'M',spring,'R');
connect_physical(mass,'M',damper,'R');
connect_physical(mass,'M',sensor,'R');
% Road node: both lower ends are attached to the imposed-moving base.
connect_physical(spring,'C',source,'R');
connect_physical(damper,'C',source,'R');
% Fixed reference node: road source C, sensor C and network solver.
connect_physical(source,'C',ref,'V');
connect_physical(sensor,'C',ref,'V');
connect_physical(solver,'port',ref,'V');
% Physical-signal connections.
connect_physical(psIn,'output',source,'S');
connect_physical(sensor,'P',psOut,'input');

% Ordinary Simulink signal lines.
add_line(model,'Road_velocity_input/1','Simulink_to_physical/1','autorouting','on');
add_line(model,'Physical_to_Simulink/1','Compare_body_and_road/1','autorouting','on');
add_line(model,'Road_height_plot/1','Compare_body_and_road/2','autorouting','on');
add_line(model,'Compare_body_and_road/1','Scope/1','autorouting','on');
add_line(model,'Physical_to_Simulink/1','Body_displacement_data/1','autorouting','on');

set_param(model,'StopTime',num2str(simTime),'Solver','ode15s', ...
    'MaxStep','0.005','RelTol','1e-5');
save_system(model);

%% 5. Simulate and read the body position measured by the sensor
simulation = sim(model);
bodyTS = simulation.get('susp_body_x');
tBody = bodyTS.Time(:);
xBody = squeeze(bodyTS.Data);
xBody = xBody(:);
xOnGrid = interp1(tBody,xBody,t,'linear','extrap');

figure('Name','Suspension: road input versus car-body motion','Color','w');
plot(t,100*r,'k--','LineWidth',1.6); hold on;
plot(t,100*xOnGrid,'b-','LineWidth',2);
ylabel('Vertical displacement (cm)'); xlabel('Time (s)');
legend('Road / wheel (r)','Car body (x)','Location','best');
title(sprintf('m = %g kg   |   k = %g N/m   |   c = %g N s/m',m,k,c));
grid on; xlim([0 simTime]);

%% 6. Animated educational schematic (not a 3-D mechanical animation)
if showAnimation
    animate_suspension(t,r,xOnGrid,m,k,c,bumpHeight);
end
fprintf('\nCreated and saved: %s.slx\n',model);
fprintf('Model:  m*x_ddot + c*(x_dot-r_dot) + k*(x-r) = 0\n');
fprintf('Change m, k, or c at the start of this file, and run again.\n');
end

function connect_physical(block1,requested1,block2,requested2)
% MATLAB R2023a has no simscape.addConnection. Use physical port handles.
% Port positions identify the displayed R/C/S/P terminal labels.
ph1 = physical_port_handle(block1, requested1);
ph2 = physical_port_handle(block2, requested2);
add_line(bdroot(block1), ph1, ph2, 'autorouting','on');
end

function ph = physical_port_handle(blockPath, label)
% Works with R2023a Foundation mechanical blocks in their default
% orientation; no dependence on internal masked-port numbering.
p = get_param(blockPath,'PortHandles');
h = [p.LConn(:); p.RConn(:)];
if isempty(h)
    error('No physical ports found on block: %s', blockPath);
end
xy = zeros(numel(h),2);
for j = 1:numel(h)
    xy(j,:) = get_param(h(j),'Position');
end
[~,blockName] = fileparts(blockPath);
% Blocks with a single physical conserving/signal port:
if numel(h) == 1
    ph = h(1); return;
end
switch blockName
    case {'Spring','Damper'}
        % R and C are left and right, respectively.
        if strcmpi(label,'R'), [~,idx] = min(xy(:,1));
        else, [~,idx] = max(xy(:,1)); end
    case 'Road_velocity_source'
        % Ideal velocity source: S on left, R at top, C at bottom.
        [~,idxS] = min(xy(:,1));
        others = setdiff(1:numel(h),idxS);
        if strcmpi(label,'S')
            idx = idxS;
        elseif strcmpi(label,'R')
            [~,ii] = min(xy(others,2)); idx = others(ii);
        else
            [~,ii] = max(xy(others,2)); idx = others(ii);
        end
    case 'Body_motion_sensor'
        % Motion sensor: R on left, C upper-right, P lower-right.
        [~,idxR] = min(xy(:,1));
        others = setdiff(1:numel(h),idxR);
        if strcmpi(label,'R')
            idx = idxR;
        elseif strcmpi(label,'C')
            [~,ii] = min(xy(others,2)); idx = others(ii);
        else
            [~,ii] = max(xy(others,2)); idx = others(ii);
        end
    otherwise
        error('Unexpected multiple ports on block %s; inspect port positions.',blockPath);
end
ph = h(idx);
end

function animate_suspension(t,road,body,m,k,c,h)
fig = figure('Name','Classroom animation - suspension','Color','w', ...
    'Position',[160 90 940 480]);
ax = axes('Parent',fig,'Position',[.06 .13 .44 .79]);
curveAx = axes('Parent',fig,'Position',[.59 .17 .36 .7]);
plot(curveAx,t,100*road,'k--','LineWidth',1.5); hold(curveAx,'on');
plot(curveAx,t,100*body,'b-','LineWidth',1.7);
marker = xline(curveAx,0,':','Color',[.85 .25 .2]);
legend(curveAx,{'Road','Body'},'Location','northeast');
ylabel(curveAx,'Displacement (cm)'); xlabel(curveAx,'Time (s)');
title(curveAx,'Input and response'); grid(curveAx,'on');
xlim(curveAx,[0 t(end)]);

% Display motion with a moderate vertical magnification for visibility.
scale = 3.5;
frameIdx = unique([1:20:length(t),length(t)]);
for j = 1:numel(frameIdx)
    if ~ishandle(fig), return; end
    i = frameIdx(j);
    yRoad = 0.35 + scale*road(i);
    yBody = 1.9 + scale*body(i);
    cla(ax); hold(ax,'on');
    rectangle(ax,'Position',[0.45,yBody,2.1,.42], ...
        'FaceColor',[.65 .76 .94],'EdgeColor',[.16 .33 .65], ...
        'LineWidth',2,'Curvature',.10);
    text(ax,1.5,yBody+.21,sprintf('Car body   m = %g kg',m), ...
        'HorizontalAlignment','center','FontWeight','bold','FontSize',10);
    plot(ax,[.25 2.85],[yRoad yRoad],'k-','LineWidth',6);
    plot(ax,[.4 2.7],[yRoad-.12 yRoad-.12],'-','Color',[.6 .6 .6]);
    % Physical spring between the moving base and moving car body.
    xs = 1.05 + .14*[0 0 1 -1 1 -1 1 -1 1 -1 0 0];
    ys = linspace(yRoad,yBody,12);
    plot(ax,xs,ys,'Color',[.12 .4 .84],'LineWidth',2.5);
    % Physical damper: two-part piston, cylinder and connecting rod.
    plot(ax,[2.08 2.08],[yRoad yRoad+.4*(yBody-yRoad)], ...
        'Color',[.87 .42 .08],'LineWidth',2.5);
    rectangle(ax,'Position',[1.94,yRoad+.25*(yBody-yRoad), ...
        .28,.27*(yBody-yRoad)],'EdgeColor',[.87 .42 .08], ...
        'LineWidth',2,'FaceColor',[1 .88 .72]);
    plot(ax,[2.08 2.08],[yRoad+.45*(yBody-yRoad) yBody], ...
        'Color',[.87 .42 .08],'LineWidth',2);
    text(ax,.76,(yBody+yRoad)/2,'k','Color',[.12 .4 .84],'FontSize',13);
    text(ax,2.30,(yBody+yRoad)/2,'c','Color',[.87 .42 .08],'FontSize',13);
    title(ax,sprintf('t = %.2f s   |   x = %.1f cm   |   r = %.1f cm', ...
        t(i),100*body(i),100*road(i)));
    xlabel(ax,sprintf('Spring = %g N/m   |   Damper = %g N s/m',k,c));
    axis(ax,[0 3.1 0 3.1]); axis(ax,'equal');
    set(ax,'XTick',[],'YTick',[],'Box','on');
    marker.Value = t(i);
    drawnow;
    pause(.014);
end
end
