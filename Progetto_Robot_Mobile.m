%parte_1;
parte_2;

%% =================================================
% PARTE 1
%% =================================================
function parte_1()
    clear; close all; clc;

    % -------------------------------------------------
    % 2. PARAMETRI GEOMETRICI
    % -------------------------------------------------
    r = 0.1;        % raggio della ruota [m]
    d = 0.5;        % distanza tra le ruote [m]
    Tsim = 20;

    % -------------------------------------------------
    % 3. STATO INIZIALE
    % -------------------------------------------------
    q0 = [0; 0; 0];   % [x y theta]

    % -------------------------------------------------
    % 4. COMANDI
    % -------------------------------------------------
    robot_params = struct();
    wR = 1;     % [rad/s]
    wL = 0;     % [rad/s]

    robot_params.comandi = [wR; wL];
    robot_params.parametri = [r, d];

    % -------------------------------------------------
    % 5. SIMULAZIONE
    % -------------------------------------------------
    [t, q] = ode45(@(t,q) robot_model(t, q, robot_params), [0 Tsim], q0);

    % -------------------------------------------------
    % 6. TRAIETTORIA
    % -------------------------------------------------
    figure;
    plot(q(:, 1), q(:, 2), 'b', 'LineWidth',2);
    grid on; axis equal;
    xlabel('x[m]'); ylabel('y[m]');
    title('Traiettoria (open-loop, wR/wL costanti)');

    figure;
    subplot(3, 1, 1);
    plot(t, q(:, 1), 'LineWidth', 2), grid on;
    ylabel ('x[m]'); title('Stato vs tempo');

    subplot(3, 1, 2);
    plot(t, q(:, 2), 'LineWidth', 2), grid on;
    ylabel ('y[m]');

    subplot(3, 1, 3);
    plot(t, q(:, 3), 'LineWidth', 2), grid on;
    ylabel('\theta [rad]'); xlabel('Tempo [sec]');

    % -------------------------------------------------
    % 7. ANIMAZIONE
    % -------------------------------------------------
    figure;
    axis equal; grid on;
    xlabel('x [m]'); ylabel('y [m]');
    hold on;

    for k = 1:length(t)
        plot(q(1:k, 1),q(1:k, 2), 'b-', 'linewidth', 1);

        px = q(k, 1); py = q(k, 2); th = q(k, 3);

        robot_r = 0.2;
        p1 = [px + robot_r * cos(th);
              py + robot_r * sin(th)];
        p2 = [px + robot_r * cos(th + 2*pi/3);
              py + robot_r * sin(th + 2*pi/3)];
        p3 = [px + robot_r * cos(th + 4*pi/3);
              py + robot_r * sin(th + 4*pi/3)];

        fill([p1(1) p2(1) p3(1)], [p1(2) p2(2) p3(2)], 'r');
        axis([-5 5 -5 5])
        drawnow;
    end
end

%% -------------------------------------------------
% FUNZIONE MODELLO CINEMATICO
%% -------------------------------------------------
function dq = robot_model(~, q, robot_params)
    dq = zeros(3, 1);

    theta = q(3);

    wR = robot_params.comandi(1);
    wL = robot_params.comandi(2);

    r = robot_params.parametri(1);
    d = robot_params.parametri(2);

    v = (r/2)*(wR + wL);
    w = (r/d)*(wR - wL);

    dq(1) = v*cos(theta);
    dq(2) = v*sin(theta);
    dq(3) = w;
end

%% =================================================
% PARTE 2
%% =================================================
function parte_2()
    clc; clear; close all;

    % -------------------------------------------------
    % 1. DEFINIZIONE DELLA MAPPA
    % -------------------------------------------------
    mapSize = [40 40];
    map = zeros(mapSize);

    map(10:30,20) = 1;
    map(15, 5:25) = 1;
    map(25, 3:38) = 1;

    start = [2 2];
    goal  = [38 25];

    figure('Name','Occupancy Grid');
    imagesc(map'); axis equal tight;
    colormap(gray);
    set(gca,'YDir','normal'); hold on;
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1),  goal(2),  'ro', 'MarkerSize', 10, 'LineWidth', 2);
    title('Occupancy Grid Map');
    legend('Start','Goal');

    % -------------------------------------------------
    % 2. COSTRUZIONE DEL C-SPACE (INFLAZIONE OSTACOLI)
    % -------------------------------------------------
    robotRadius = 1;  % celle
    se = strel('disk', robotRadius);
    mapInflated = imdilate(map, se);

    figure('Name','C-space');
    imagesc(mapInflated'); axis equal tight;
    colormap(gray);
    set(gca,'YDir','normal'); hold on;
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1),  goal(2),  'ro', 'MarkerSize', 10, 'LineWidth', 2);
    title('Spazio delle configurazioni');
    legend('Start','Goal');

    % -------------------------------------------------
    % 3. PIANIFICAZIONE (Dijkstra su griglia)
    % -------------------------------------------------
    [path, distMap] = cellDecompositionDijkstra(mapInflated, start, goal);

    if isempty(path)
        error('Nessun path trovato.');
    end

    % -------------------------------------------------
    % 4. VISUALIZZAZIONE RISULTATO
    % -------------------------------------------------
    figure('Name','Pianificazione (Dijkstra)');
    imagesc(mapInflated'); axis equal tight;
    colormap(gray);
    set(gca,'YDir','normal'); hold on;
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1),  goal(2),  'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
    title('Path pianificato');
    legend('Ostacoli (inflated)','Start','Goal','Path');

    figure('Name','Dijkstra - mappa distanze');
    imagesc(distMap'); axis equal tight;
    set(gca,'YDir','normal'); colorbar;
    title('Distanza minima dallo Start (celle raggiunte)');

    % -------------------------------------------------
    % 5. CONTROLLO + SIMULAZIONE (NUOVO)
    % -------------------------------------------------
    % Parametri robot differenziale
    r = 0.1;     % [m]
    d = 0.5;     % [m]

    cellSize = 1.0;
    wp = path * cellSize; % Nx2: [x y]

    % Stato iniziale coerente con start
    q0 = [start(1)*cellSize; start(2)*cellSize; 0];

    % Guadagni controllore
    ctrl = struct();
    ctrl.k_rho   = 0.8;     % velocità in funzione della distanza
    ctrl.k_alpha = 2.0;     % velocità angolare in funzione dell'errore angolare
    ctrl.v_max   = 1.0;     % saturazione v [m/s]
    ctrl.w_max   = 3.0;     % saturazione omega [rad/s]
    ctrl.eps_wp  = 0.30;    % soglia raggiungimento waypoint [m]

    sim = struct();
    sim.r = r; sim.d = d;
    sim.ctrl = ctrl;

    % Simulazione waypoint-by-waypoint (ode45 + Events)
    [T, Q, W, RHO] = simulate_follow_path_ode45(q0, wp, sim);

    % -------------------------------------------------
    % 6. RISULTATI RICHIESTI
    % -------------------------------------------------
    % Traiettoria pianificata vs eseguita
    figure('Name','Planned vs Executed');
    imagesc(mapInflated'); axis equal tight;
    colormap(gray);
    set(gca,'YDir','normal'); hold on;

    plot(path(:,1), path(:,2), 'b--', 'LineWidth', 2);        
    plot(Q(:,1)/cellSize, Q(:,2)/cellSize, 'r-', 'LineWidth', 2);
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1),  goal(2),  'ro', 'MarkerSize', 10, 'LineWidth', 2);
    title('Traiettoria pianificata (blu) vs eseguita (rosso)');
    legend('Planned path','Executed traj','Start','Goal');

    % Errore di tracking (rho)
    figure('Name','Tracking error');
    plot(T, RHO, 'LineWidth', 2); grid on;
    xlabel('t [s]'); ylabel('\rho [m]');
    title('Errore di tracking (distanza al waypoint corrente)');

    % Velocità ruote
    figure('Name','Wheel speeds');
    subplot(2,1,1);
    plot(T, W(:,1), 'LineWidth', 2); grid on;
    ylabel('\omega_R [rad/s]'); title('Velocità ruote');
    subplot(2,1,2);
    plot(T, W(:,2), 'LineWidth', 2); grid on;
    ylabel('\omega_L [rad/s]'); xlabel('t [s]');

    % Animazione del moto simulato + waypoint
    figure('Name','Animazione (moto simulato)');
    imagesc(mapInflated'); axis equal tight;
    colormap(gray);
    set(gca,'YDir','normal'); hold on;
    plot(path(:,1), path(:,2), 'b--', 'LineWidth', 1);
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1),  goal(2),  'ro', 'MarkerSize', 10, 'LineWidth', 2);
    title('Animazione: traiettoria simulata (differenziale)');

    robotPlot = plot(Q(1,1)/cellSize, Q(1,2)/cellSize, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    trajPlot  = plot(Q(1,1)/cellSize, Q(1,2)/cellSize, 'r-', 'LineWidth', 1);

    for k = 1:length(T)
        set(robotPlot, 'XData', Q(k,1)/cellSize, 'YData', Q(k,2)/cellSize);
        set(trajPlot,  'XData', Q(1:k,1)/cellSize, 'YData', Q(1:k,2)/cellSize);
        drawnow;
    end
end

%% -------------------------------------------------
% DIJKSTRA SU GRIGLIA
%% -------------------------------------------------
function [path, distMap] = cellDecompositionDijkstra(map, start, goal)

    mapSize = size(map);
    rows = mapSize(1); cols = mapSize(2);

    if map(start(1), start(2)) == 1
        error('Start in ostacolo (dopo inflazione). Cambia start o robotRadius.');
    end
    if map(goal(1), goal(2)) == 1
        error('Goal in ostacolo (dopo inflazione). Cambia goal o robotRadius.');
    end

    distMap = inf(rows, cols);
    visited = false(rows, cols);
    parent = zeros(rows, cols, 2);

    distMap(start(1), start(2)) = 0;

    moves = [ 1 0;
             -1 0;
              0 1;
              0 -1];

    while true
        minDist = inf;
        current = [-1 -1];
        for r = 1:rows
            for c = 1:cols
                if ~visited(r,c) && distMap(r,c) < minDist
                    minDist = distMap(r,c);
                    current = [r c];
                end
            end
        end

        if current(1) == -1
            path = [];
            return;
        end

        if isequal(current, goal)
            break;
        end

        visited(current(1), current(2)) = true;

        for i = 1:size(moves,1)
            nxt = current + moves(i,:);

            if nxt(1) < 1 || nxt(2) < 1 || nxt(1) > rows || nxt(2) > cols
                continue;
            end

            if map(nxt(1), nxt(2)) == 1 || visited(nxt(1), nxt(2))
                continue;
            end

            newCost = distMap(current(1), current(2)) + 1;

            if newCost < distMap(nxt(1), nxt(2))
                distMap(nxt(1), nxt(2)) = newCost;
                parent(nxt(1), nxt(2), :) = current;
            end
        end
    end

    path = goal;
    while ~isequal(path(1,:), start)
        p = parent(path(1,1), path(1,2), :);
        p = reshape(p, 1, 2);

        if isequal(p, [0 0])
            path = [];
            return;
        end

        path = [p; path];
    end
end

% =========================================================
% SIMULAZIONE 
% =========================================================
function [T, Q, W, RHO] = simulate_follow_path_ode45(q0, wp, sim)
    r = sim.r; d = sim.d;
    ctrl = sim.ctrl;

    T = [];
    Q = [];
    W = [];
    RHO = [];

    t0 = 0;
    q_init = q0;

    for i = 2:size(wp,1)
        target = wp(i,:);

        odeOpts = odeset( ...
            'RelTol', 1e-6, 'AbsTol', 1e-8, ...
            'Events', @(t,q) event_reach_waypoint(t, q, target, ctrl.eps_wp) );

        % Integra finché non raggiunge il waypoint (evento) o finché ode45 non si ferma
        [t_seg, q_seg] = ode45(@(t,q) closed_loop_dynamics(t, q, target, r, d, ctrl), [t0 t0+50], q_init, odeOpts);

        % Controlli/errore
        w_seg = zeros(length(t_seg), 2);
        rho_seg = zeros(length(t_seg), 1);
        for k = 1:length(t_seg)
            [wR, wL, rho] = controller_to_wheels(q_seg(k,:)', target, r, d, ctrl);
            w_seg(k,:) = [wR, wL];
            rho_seg(k) = rho;
        end

        % Evita duplicati
        if ~isempty(T)
            t_seg = t_seg(2:end);
            q_seg = q_seg(2:end,:);
            w_seg = w_seg(2:end,:);
            rho_seg = rho_seg(2:end,:);
        end

        T = [T; t_seg];
        Q = [Q; q_seg];
        W = [W; w_seg];
        RHO = [RHO; rho_seg];

        % Prepara prossimo segmento
        t0 = T(end);
        q_init = Q(end,:)';
    end
end

function dq = closed_loop_dynamics(~, q, target, r, d, ctrl)
    % Calcola wR,wL dal controllore
    robot_params = struct();
    [wR, wL] = controller_to_wheels(q, target, r, d, ctrl);
    robot_params.comandi = [wR; wL];
    robot_params.parametri = [r, d];

    dq = robot_model([], q, robot_params);
end

function [value, isterminal, direction] = event_reach_waypoint(~, q, target, eps_wp)
    dx = target(1) - q(1);
    dy = target(2) - q(2);
    rho = sqrt(dx*dx + dy*dy);
    value = rho - eps_wp;  % evento quando value = 0
    isterminal = 1;        % ferma integrazione
    direction = -1;        % deve attraversare da positivo a negativo
end

% =========================================================
% Controllore
% =========================================================
function [wR, wL, rho] = controller_to_wheels(q, target, r, d, ctrl)
    x = q(1); y = q(2); th = q(3);

    dx = target(1) - x;
    dy = target(2) - y;

    rho = sqrt(dx*dx + dy*dy);
    desiredHeading = atan2(dy, dx);
    alpha = wrapToPi_custom(desiredHeading - th);

    % legge di controllo
    v = ctrl.k_rho * rho;
    w = ctrl.k_alpha * alpha;

    % saturazioni
    v = clamp(v, -ctrl.v_max, ctrl.v_max);
    w = clamp(w, -ctrl.w_max, ctrl.w_max);

    % conversione v,w -> velocità ruote
    % v = (r/2)(wR+wL),  w = (r/d)(wR-wL)
    wR = (1/r) * (v + (d/2)*w);
    wL = (1/r) * (v - (d/2)*w);
end

function y = clamp(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

function ang = wrapToPi_custom(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
