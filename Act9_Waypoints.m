%% Differential drive vehicle following waypoints from CSV

%% Define Vehicle
R = 0.1;
L = 0.5;
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;
tVec = 0:sampleTime:150;

%% Leer CSV
data = readtable('jaguar_Majo.csv');

% Extraer columnas x y y
waypoints = [data.x data.y];

%% Verificar puntos
figure
plot(waypoints(:,1),waypoints(:,2),'o-')
axis equal
grid on
title('Jaguar cargado desde CSV')

%% Pose inicial
initPose = [waypoints(1,1);
            waypoints(1,2);
            0];

pose = zeros(3,numel(tVec));
pose(:,1) = initPose;

%% Visualizador
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Controlador
controller = controllerPurePursuit;

controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.3;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 3;

%% Simulación
close all
r = rateControl(1/sampleTime);

for idx = 2:numel(tVec)

    [vRef,wRef] = controller(pose(:,idx-1));

    [wL,wR] = inverseKinematics(dd,vRef,wRef);

    [v,w] = forwardKinematics(dd,wL,wR);

    velB = [v;0;w];
    vel = bodyToWorld(velB,pose(:,idx-1));

    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    viz(pose(:,idx),waypoints)

    waitfor(r)

end