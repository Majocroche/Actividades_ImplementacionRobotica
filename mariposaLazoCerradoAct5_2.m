%Limpieza de pantalla
clc

%% 1. TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 93.99;
ts = 0.05;
t  = 0:ts:tf;
N  = length(t);

%% 2. WAYPOINTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cada fila: [xd, yd, tolerancia]
% Las ganancias se calculan solas igual que antes, aquí solo defines destinos
waypoints = [
     0.0,    1.5,  0.12;
     1.0,    2.0,  0.12;
     0.5,    2.5,  0.12;
     0.0,    3.5,  0.12;
     0.5,    4.5,  0.12;
     1.5,    4.5,  0.12;
     2.5,    4.0,  0.12;
     0.5,    4.5,  0.12;
     3.5,    5.0,  0.12;
     0.5,    4.5,  0.12;
     3.0,    4.5,  0.12;
     2.5,    5.0,  0.12;
     3.0,    4.5,  0.12;
     3.5,    4.0,  0.12;
     4.5,    4.5,  0.12;
     5.5,    4.5,  0.12;
     6.0,    3.5,  0.12;
     5.5,    2.5,  0.12;
     5.0,    2.0,  0.12;
     6.0,    1.5,  0.12;
     6.0,    1.0,  0.12;
     5.0,    0.5,  0.12;
     4.5,    1.0,  0.12;
     3.5,    1.5,  0.12;
     3.5,    4.0,  0.12;
     3.5,    1.5,  0.12;
     3.0,    1.0,  0.12;
     2.5,    1.5,  0.12;
     2.5,    4.0,  0.12;
     2.5,    1.5,  0.12;
     0.5,    0.5,  0.12;
     0.0,    1.0,  0.12;
     
    
];
numWP  = size(waypoints, 1);
wpIdx  = 1;

%% 3. CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1(1)   = 0;
y1(1)   = 1;
phi(1)  = pi/2;
hx(1)   = x1(1);
hy(1)   = y1(1);

%% 4. PARÁMETROS CONTROL ADAPTATIVO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (exactamente igual que tu código original, sin cambios)
Kv0     = 0.8;
Kw0     = 1.5;
alpha_v = 0.08;
alpha_w = 0.05;

Kv_max  = 2.5;   Kw_max = 3.5;
Kv_min  = 0.3;   Kw_min = 0.5;

v_max   = 1.2;
w_max   = 1.5;

v_prev  = 0;
w_prev  = 0;
beta    = 0.85;

%% 5. BUCLE DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = 1:N

    % --- Waypoint activo ---
    hxd = waypoints(wpIdx, 1);
    hyd = waypoints(wpIdx, 2);
    tol = waypoints(wpIdx, 3);

    %% a) ERRORES
    hxe(k) = hxd - hx(k);
    hye(k) = hyd - hy(k);

    Error(k) = sqrt(hxe(k)^2 + hye(k)^2);

    phi_d  = atan2(hye(k), hxe(k));
    e_phi  = phi_d - phi(k);
    e_phi  = atan2(sin(e_phi), cos(e_phi));   % normalización angular

    %% b) GANANCIAS AUTO-SINTONIZABLES (sin cambios)
    Kv(k) = Kv0 + alpha_v * Error(k);
    Kw(k) = Kw0 + alpha_w * abs(e_phi);

    Kv(k) = min(max(Kv(k), Kv_min), Kv_max);
    Kw(k) = min(max(Kw(k), Kw_min), Kw_max);

    %% c) CONTROLADOR (sin cambios)
    v_raw = Kv(k) * Error(k);
    v_raw = v_raw * cos(e_phi);       % reduce v si hay mucho error angular
    w_raw = Kw(k) * e_phi;

    %% d) SATURACIÓN DE VELOCIDADES (sin cambios)
    v_raw = max(min(v_raw,  v_max), -v_max);
    w_raw = max(min(w_raw,  w_max), -w_max);

    %% e) FILTRO SUAVIZANTE (sin cambios)
    v(k) = beta * v_prev + (1 - beta) * v_raw;
    w(k) = beta * w_prev + (1 - beta) * w_raw;
    v_prev = v(k);
    w_prev = w(k);

    %% f) CAMBIO DE WAYPOINT
    wpLog(k) = wpIdx;
    if Error(k) <= tol && wpIdx < numWP
        wpIdx = wpIdx + 1;
        % Reset del filtro para arranque limpio hacia el nuevo punto
        v_prev = 0;
        w_prev = 0;
    end

    %% g) MODELO CINEMÁTICO (sin cambios)
    phi(k+1)  = phi(k)  + w(k) * ts;
    xp1       = v(k) * cos(phi(k));
    yp1       = v(k) * sin(phi(k));
    x1(k+1)   = x1(k)  + xp1 * ts;
    y1(k+1)   = y1(k)  + yp1 * ts;
    hx(k+1)   = x1(k+1);
    hy(k+1)   = y1(k+1);
end

%% 6. SIMULACIÓN 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene = figure;
set(scene, 'Color', 'white');
set(gca,   'FontWeight', 'bold');
sizeScreen = get(0, 'ScreenSize');
set(scene, 'position', sizeScreen);
camlight('headlight');
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view([-0.1 35]);
axis([-11 11 -11 11 0 1]);

scale = 4;
MobileRobot_5;
hold on;
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale);

% Dibujar todos los waypoints numerados

H2 = plot3(hx(1), hy(1), 0, 'r', 'LineWidth', 2);
     plot3(hx(1), hy(1), 0, 'go', 'LineWidth', 2);   % posición inicial

%% Animación
for k = 1:N
    delete(H1); delete(H2);
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    H2 = plot3(hx(1:k), hy(1:k), zeros(1,k), 'r', 'LineWidth', 2);
    title(sprintf('WP activo: %d/%d  |  Error: %.3f m', wpLog(k), numWP, Error(k)));
    pause(ts);
end

%% 7. GRÁFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph = figure;
set(graph, 'position', sizeScreen);

subplot(4,1,1)
plot(t, v, 'b', 'LineWidth', 2), grid on
xlabel('Tiempo [s]'); ylabel('[m/s]'); legend('Velocidad lineal');

subplot(4,1,2)
plot(t, w, 'g', 'LineWidth', 2), grid on
xlabel('Tiempo [s]'); ylabel('[rad/s]'); legend('Velocidad angular');

subplot(4,1,3)
plot(t, Error, 'r', 'LineWidth', 2), grid on
xlabel('Tiempo [s]'); ylabel('[m]'); legend('Error de posición');

subplot(4,1,4)
stairs(t, wpLog, 'k', 'LineWidth', 2), grid on
xlabel('Tiempo [s]'); ylabel('Índice'); legend('Waypoint activo');
yticks(1:numWP);
ylim([0.5, numWP + 0.5]);