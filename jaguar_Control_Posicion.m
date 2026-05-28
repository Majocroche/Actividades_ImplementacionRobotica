%Limpieza de pantalla
clear all
close all
clc

%% MODIFICADO - Leer puntos desde CSV %%%%%%%%%%%%%%%%%%%%%%%%%%%%
datos = readmatrix('jaguar_Majo.csv'); % Lee el archivo CSV
waypoints = datos(:, 2:3); % %% MODIFICADO - X en columna 2, Y en columna 3
num_waypoints = size(waypoints, 1);    % Número total de puntos
idx_wp = 1;                            % Índice del waypoint actual

fprintf('Waypoints cargados: %d puntos\n', num_waypoints);

%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 10 * num_waypoints; % Tiempo total según cantidad de puntos
ts = 0.1;
t  = 0:ts:tf;
N  = length(t);

%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1(1)   = waypoints(1,1); % Posición inicial desde el CSV
y1(1)   = waypoints(1,2);
phi(1)  = pi/2;

%% MODIFICADO - Primer objetivo es el segundo waypoint %%%%%%%%%%%%%
hxd = waypoints(2,1);
hyd = waypoints(2,2);
idx_wp = 2;

%3 POSICIÓN INICIAL DEL PUNTO DE CONTROL
hx(1) = x1(1);
hy(1) = y1(1);

tolerancia = 0.3; % Distancia mínima para considerar que llegó al punto

%4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = 1:N

    %% MODIFICADO - Verificar si llegó al waypoint actual %%%%%%%%%%
    dist_actual = sqrt((hx(k)-hxd)^2 + (hy(k)-hyd)^2);

    if dist_actual < tolerancia && idx_wp < num_waypoints
        idx_wp = idx_wp + 1;           % Avanzar al siguiente punto
        hxd = waypoints(idx_wp, 1);
        hyd = waypoints(idx_wp, 2);
        fprintf('Waypoint %d alcanzado → siguiente: (%.1f, %.1f)\n', ...
                idx_wp-1, hxd, hyd);
    end

    % a) Errores de control
    hxe(k) = hxd - hx(k);
    hye(k) = hyd - hy(k);
    he = [hxe(k); hye(k)];
    Error(k) = sqrt(hxe(k)^2 + hye(k)^2);

    % b) Matriz Jacobiana
    J = [cos(phi(k)) -sin(phi(k));
         sin(phi(k))  cos(phi(k))];

    % c) Ganancias
    K = [0.2 0; 0 0.2];

    % d) Ley de Control
    qpRef = pinv(J) * K * he;
    v(k)  = qpRef(1);
    w(k)  = qpRef(2);

%5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    phi(k+1) = phi(k) + w(k)*ts;

    xp1 = v(k)*cos(phi(k));
    yp1 = v(k)*sin(phi(k));

    x1(k+1) = x1(k) + xp1*ts;
    y1(k+1) = y1(k) + yp1*ts;

    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%
scene = figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen = get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view(2); % %% MODIFICADO - Vista 2D desde arriba para ver bien la trayectoria

%% MODIFICADO - Límites automáticos según los waypoints del CSV
margen = 2;
min_x = min(waypoints(:,1)) - margen;
max_x = max(waypoints(:,1)) + margen;
min_y = min(waypoints(:,2)) - margen;
max_y = max(waypoints(:,2)) + margen;
axis([min_x max_x min_y max_y 0 1]);

scale = 4;
MobileRobot_5;
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;

%% MODIFICADO - Línea de trayectoria con z=0 explícito y color visible
H2 = plot3(hx(1),hy(1),0.01,'r-','lineWidth',3); % z=0.01 para que no quede "debajo" del piso

% Graficar TODOS los waypoints
for i = 1:num_waypoints
    plot3(waypoints(i,1), waypoints(i,2), 0.01, 'bo', 'MarkerSize', 10, 'lineWidth', 2);
    text(waypoints(i,1)+0.2, waypoints(i,2)+0.2, 0.01, ...
         sprintf('P%d',i), 'FontWeight','bold', 'FontSize', 10);
end

% Línea punteada que conecta los waypoints (ruta planeada)
plot3(waypoints(:,1), waypoints(:,2), ones(num_waypoints,1)*0.01, ...
      'b--', 'lineWidth', 1.5);

H4 = plot3(hx(1),hy(1),0.01,'go','MarkerSize',10,'lineWidth',2);

step = 1;
for k = 1:step:N
    delete(H1);
    delete(H2);

    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    
    %% MODIFICADO - z como vector de 0.01 para que la línea sea visible
    H2 = plot3(hx(1:k), hy(1:k), ones(1,k)*0.01, 'r-', 'lineWidth', 3);
    
    drawnow; % %% MODIFICADO - fuerza actualización inmediata de la figura
    pause(ts);
end