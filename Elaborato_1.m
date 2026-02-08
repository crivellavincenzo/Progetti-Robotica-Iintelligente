clear; clc; close all;

% Avvio Progetto_1
%val = 1;
% Avvio Progetto_2
%val = 2;
% Avvio Progetto_3
val = 3;

if val == 1
    progetto_1;
end 

if val == 2
    progetto_2;
end

if val == 3
    progetto_3;
end

%% --- PROGETTO 1
function progetto_1()
    % --- Parametri del robot ---

    L = [1, 1, 1, 1];
    
    % --- Finestra grafica ---
    figure; axis equal; grid on; hold on;
    xlim([-sum(L) sum(L)]);
    ylim([-sum(L) sum(L)]);
    xlabel('x'); ylabel('y');
    title('Animazione Robot 4R Planare con Terne DH');
    scale = 0.25;
    
    % --- Traiettoria dei giunti ---
    
    Tmax = 200;
    theta1 = linspace(0, pi,   Tmax);
    theta2 = linspace(0, pi/2, Tmax);
    theta3 = linspace(0, pi/2, Tmax);
    theta4 = linspace(0, pi/2, Tmax);   % aggiunto 4° giunto
    
    % --- Loop animazione ---
    for i = 1:Tmax
        cla; hold on; grid on;
        xlim([-sum(L) sum(L)]);
        ylim([-sum(L) sum(L)]);
    
        theta = [theta1(i) theta2(i) theta3(i) theta4(i)];
        
        % comparare i risultati con il metodo geometrico e DH
        [x_geo, y_geo, phi_geo] = fk_4R(theta, L);
        [T, x, y, phi] = fk_4R_DH(theta, L);
    
        disp('errore nel calcolo cinematica diretta:')
        disp("x: "+(x_geo-x))
        disp("y: "+(y_geo-y))
        disp("phi: "+(phi_geo-phi))
    
        % --- DH: matrici A_i e trasformazioni T_i ---
        A1 = dh_matrix(L(1), 0, 0, theta(1));
        A2 = dh_matrix(L(2), 0, 0, theta(2));
        A3 = dh_matrix(L(3), 0, 0, theta(3));
        A4 = dh_matrix(L(4), 0, 0, theta(4));
    
        T0 = eye(4);
        T1 = A1;
        T2 = A1*A2;
        T3 = A1*A2*A3;
        T4 = A1*A2*A3*A4;  % = T
    
        p0 = T0(1:2, 4);
        p1 = T1(1:2, 4);
        p2 = T2(1:2, 4);
        p3 = T3(1:2, 4);
        p4 = T4(1:2, 4);
    
        % --- Plot robot ---
        plot([p0(1) p1(1) p2(1) p3(1) p4(1)], ...
             [p0(2) p1(2) p2(2) p3(2) p4(2)], ...
             'ko-', 'LineWidth', 2);
    
        % --- Terne DH ---
        drawFrame2D(T0, scale, '0');
        drawFrame2D(T1, scale, '1');
        drawFrame2D(T2, scale, '2');
        drawFrame2D(T3, scale, '3');
        drawFrame2D(T4, scale, '4');
    
        drawnow;
    end
end

%% --- PROGETTO 2
function progetto_2()
    % --- Parametri del robot ---
    L = [1, 1, 1, 1];
    
    %  --- Finestra grafica ---
    figure;
    axis equal; grid on; hold on;
    xlim([-sum(L) sum(L)]);
    ylim([-sum(L) sum(L)]);
    xlabel('x'); ylabel('y');
    title('Robot Planare 4R - IK Jacobiano');
    
    % Traiettoria cartesiana da seguire
    Tmax = 200;
    t = linspace(0, 2*pi, Tmax);
    
    r = 2.0;
    xc = r*cos(t);
    yc = r*sin(t);
    phi_des = pi/2 * ones(1, Tmax);   % orientazione desiderata
    % phi_des = linspace(0, pi/2, Tmax);
    % phi_des = pi/4 * ones(1, Tmax);
    
    % vettori degli errori di calcolo
    errB = zeros(1, Tmax);
    
    % Loop principale (animazione)
    for i = 1:Tmax
        cla; hold on; grid on;
        xlim([-sum(L) sum(L)]);
        ylim([-sum(L) sum(L)]);
    
        % Target corrente
        xd = xc(i);
        yd = yc(i);
        phid = phi_des(i);
    
        plot(xd, yd, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    
        theta_B = ik_4R_jacobiana(xd, yd, phid, L);               % 4x1
    
        % Se la jacobiana restituisce 4x1, il plot vuole 1x4:
        theta_B_row = theta_B(:).';  % 1x4
    
        color = [0.6 0 0];
        plot_4R(theta_B_row, L, color);
    
        title(sprintf('Target: (%.2f, %.2f)', xd, yd));
    
        
        % Forward kinematics (errore su posizione)
        [~, xB, yB, ~] = fk_4R_DH(theta_B_row, L);
        errB(i) = hypot(xB-xd, yB-yd);
    
        drawnow;
    end
    
    % Plot dell'errore di calcolo
    figure;
    plot(errB, 'r', 'LineWidth', 2);
    xlabel('tempo della simulazione');
    ylabel('errore [m]');
    legend('IK Jacobiano (pinv)');
    title('Confronto errore nel calcolo IK - 4R');
end

%% --- PROGETTO 3
function progetto_3()
    % PARAMETRI ROBOT 4R (planare, link uniformi)
    L = [1 1 1 1];          % [m] lunghezze link
    m = [1 1 1 1];          % [kg] masse link
    l = L/2;                % [m] distanza COM dal giunto (link uniformi)
    I = (1/12) .* m .* (L.^2); % [kg*m^2] inerzia al COM (asta uniforme, asse z)
    g = 9.81;               % [m/s^2] gravità

    % MODALITÀ DI CONTROLLO
    CONTROL_MODE = "JOINT"; % "TASK" oppure "JOINT"
    disp("Modalità di controllo attiva: " + CONTROL_MODE);

    % Gain joint-space (4R)
    Kp = diag([80 80 80 80]);   % proporzionale
    Kd = diag([20 20 20 20]);   % derivativa

    % Gain task-space (solo x,y)
    Kp_t = diag([100 100]);
    Kd_t = diag([20 20]);

    % TRAIETTORIA DESIDERATA (operativa)
    dt = 0.002;
    t = 0:dt:10;

    radius = 0.4;
    xc = 1.2;
    yc = 0.4;
    omega = 0.15;                   % velocità angolare

    xd_traj = zeros(2, length(t));  % posizioni desiderate
    dxd_traj = zeros(2, length(t)); % velocità desiderata

    for i = 1:length(t)
        xd_traj(:, i) = [xc + radius*cos(omega*t(i));
                         yc + radius*sin(omega*t(i))];

        dxd_traj(:, i) = [-radius*omega*sin(omega*t(i));
                          radius*omega*cos(omega*t(i))];
    end

    % TRAIETTORIA NELLO SPAZIO DEI GIUNTI (solo se in modalità JOINT)
    qd_traj = zeros(4, length(t)); % 4 giunti
    for i = 1:length(t)
        qd_traj(:,i) = ik_4R(xd_traj(1, i), xd_traj(2, i), 0, L, qd_traj(:,max(i-1,1)));
    end

    params.L = L; params.m = m; params.l = l; params.I = I; params.g = g;

    gains.Kp   = Kp;
    gains.Kd   = Kd;
    gains.Kp_t = Kp_t;
    gains.Kd_t = Kd_t;

    traj.t   = t(:);
    traj.xd  = xd_traj;
    traj.dxd = dxd_traj;
    traj.qd  = qd_traj;

    % =========================
    % STATI INIZIALI
    % =========================
    if CONTROL_MODE == "JOINT"
        q0  = zeros(4, 1);     % angoli iniziali
        dq0 = zeros(4, 1);     % velocità iniziali
    else
        % partenza coincidente con la posizione ideale (prendo il primo punto della traiettoria in giunti)
        q0  = qd_traj(:, 1);
        dq0 = zeros(4, 1);
    end

    x0 = [q0; dq0]; % stato iniziale per ode45 (8x1)

    % =========================
    % SIMULAZIONE DINAMICA CON ODE45
    % =========================
    opts = odeset('RelTol',1e-6,'AbsTol',1e-8);

    [t_ode, X] = ode45(@(tt,xx) dyn4R_ode(tt, xx, params, traj, gains, CONTROL_MODE), ...
                       [t(1) t(end)], x0, opts);

    % estrazione risultati
    Q  = X(:, 1:4).';   % 4 x N
    DQ = X(:, 5:8).';   % 4 x N

    % =========================
    % LOG END-EFFECTOR + ERRORI + TAU
    % =========================
    N = length(t_ode);

    XE  = zeros(2, N);
    TAU = zeros(4, N);

    E_q = zeros(4, N);
    E_x = zeros(2, N);

    for k = 1:N
        qk  = Q(:,k);
        dqk = DQ(:,k);

        % end-effector
        [~, xe, ye, ~] = fk_4R_DH(qk.', L);
        XE(:,k) = [xe; ye];

        % tau + errori (usiamo stessa legge di controllo dell'ODE)
        [tau_k, e_qk, e_xk] = control_tau_4R(t_ode(k), qk, dqk, params, traj, gains, CONTROL_MODE, xe, ye);
        TAU(:,k) = tau_k;

        if CONTROL_MODE == "JOINT"
            E_q(:,k) = e_qk;
        else
            E_x(:,k) = e_xk;
        end
    end

    % =========================
    % PLOT TRAIETTORIA (operativa)
    % =========================
    figure;
    plot(xd_traj(1, :), xd_traj(2, :), 'r--', 'LineWidth', 2);
    hold on; grid on; axis equal;
    plot(XE(1,:), XE(2,:), 'b', 'LineWidth', 2);
    xlabel("x"); ylabel("y");
    title("Traiettoria organo terminale");
    legend("Desiderata","Eseguita");

    % =========================
    % PLOT TRAIETTORIA DESIDERATA GIUNTI (se JOINT)
    % =========================
    figure;
    plot(traj.t, qd_traj.', 'LineWidth', 2);
    grid on;
    xlabel("Tempo");
    ylabel("Angolo [rad]");
    title("Traiettoria desiderata giunti");
    legend("giunto 1","giunto 2","giunto 3","giunto 4");

    % =========================
    % PLOT Q(t) da ode45
    % =========================
    figure;
    plot(t_ode, Q.', 'LineWidth', 2);
    grid on;
    xlabel("Tempo [s]");
    ylabel("q [rad]");
    title("Andamento giunti (ode45)");
    legend("q1","q2","q3","q4");

    % =========================
    % PLOT ERRORI
    % =========================
    if CONTROL_MODE == "JOINT"
        figure;
        plot(t_ode, E_q.', 'LineWidth', 2);
        grid on;
        xlabel("Tempo [s]");
        ylabel("Errore q [rad]");
        title("Errore ai giunti");
        legend("e_{q1}", "e_{q2}", "e_{q3}", "e_{q4}");
    else
        figure;
        plot(t_ode, E_x.', 'LineWidth', 2);
        grid on;
        xlabel("Tempo [s]");
        ylabel("Errore [m]");
        title("Errore per l'organo terminale");
        legend("e_x", "e_y");
    end

    % =========================
    % PLOT TAU(t) (RICHIESTO)
    % =========================
    figure;
    plot(t_ode, TAU.', 'LineWidth', 2);
    grid on;
    xlabel("Tempo [s]");
    ylabel("Coppia [Nm]");
    title("Coppie ai giunti");
    legend("\tau_1","\tau_2","\tau_3","\tau_4");

    % =========================
    % ANIMAZIONE
    % =========================
    figure;
    axis equal; grid on;
    xlabel("x"); ylabel("y");
    title("Animazione robot 4R (ode45)");

    step = 1; % puoi alzarlo (es 5,10) se è lento

    for i = 1:step:N
        cla;
        plot(xd_traj(1, :), xd_traj(2, :), 'r--', 'LineWidth', 2);
        hold on; grid on;

        color = [0.6 0 0];
        plot_4R(Q(:, i), L, color);
        plot(XE(1, i), XE(2, i), 'bo', 'MarkerSize', 5);

        axis([-sum(L), sum(L), -sum(L), sum(L)]);
        drawnow;
    end
end

%% --- Cinematica diretta geometrica 4R ---
function [x, y, phi] = fk_4R(theta, L)
    x = L(1)*cos(theta(1)) ...
      + L(2)*cos(theta(1)+theta(2)) ...
      + L(3)*cos(theta(1)+theta(2)+theta(3)) ...
      + L(4)*cos(theta(1)+theta(2)+theta(3)+theta(4));

    y = L(1)*sin(theta(1)) ...
      + L(2)*sin(theta(1)+theta(2)) ...
      + L(3)*sin(theta(1)+theta(2)+theta(3)) ...
      + L(4)*sin(theta(1)+theta(2)+theta(3)+theta(4));

    phi = theta(1) + theta(2) + theta(3) + theta(4);
end

%% Matrice DH standard

function A = dh_matrix(a, alpha, d, theta)
    A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
          0,           sin(alpha),            cos(alpha),             d;
          0,           0,                     0,                      1 ];
end

%% FK DH 4R
function [T, x, y, phi] = fk_4R_DH(theta, L)

    A1 = dh_matrix(L(1), 0, 0, theta(1));
    A2 = dh_matrix(L(2), 0, 0, theta(2));
    A3 = dh_matrix(L(3), 0, 0, theta(3));
    A4 = dh_matrix(L(4), 0, 0, theta(4));

    T = A1 * A2 * A3 * A4;

    x = T(1,4);
    y = T(2,4);
    phi = atan2(T(2,1), T(1,1));
end

%% --- Disegno terna 2D ---
function drawFrame2D(T, scale, label)
    origin = T(1:2, 4);
    x_ax = T(1:2, 1);
    y_ax = T(1:2, 2);

    quiver(origin(1), origin(2), scale*x_ax(1), scale*x_ax(2), 0, 'r', 'LineWidth', 1.5);
    quiver(origin(1), origin(2), scale*y_ax(1), scale*y_ax(2), 0, 'g', 'LineWidth', 1.5);
    text(origin(1)+0.15, origin(2), label, 'FontSize', 10);
end

%% Plot del robot
function plot_4R(theta, L, col)

    A1 = dh_matrix(L(1), 0, 0, theta(1));
    A2 = dh_matrix(L(2), 0, 0, theta(2));
    A3 = dh_matrix(L(3), 0, 0, theta(3));
    A4 = dh_matrix(L(4), 0, 0, theta(4));

    T0 = eye(4);
    T1 = A1;
    T2 = A1*A2;
    T3 = A1*A2*A3;
    T4 = A1*A2*A3*A4;

    p0 = T0(1:2,4);
    p1 = T1(1:2,4);
    p2 = T2(1:2,4);
    p3 = T3(1:2,4);
    p4 = T4(1:2,4);

    plot([p0(1) p1(1) p2(1) p3(1) p4(1)], ...
         [p0(2) p1(2) p2(2) p3(2) p4(2)], ...
         'o-','Color', col, 'LineWidth', 2);
end

%% Jacobiano 4R 3x4
function J = jacobian_4R(theta, L)
    t1 = theta(1); t2 = theta(2); t3 = theta(3); t4 = theta(4);
    
    s1    = sin(t1);              c1    = cos(t1);
    s12   = sin(t1+t2);           c12   = cos(t1+t2);
    s123  = sin(t1+t2+t3);        c123  = cos(t1+t2+t3);
    s1234 = sin(t1+t2+t3+t4);     c1234 = cos(t1+t2+t3+t4);
    
    L1 = L(1); L2 = L(2); L3 = L(3); L4 = L(4);
    
    J = [  -L1*s1   - L2*s12  - L3*s123 - L4*s1234,  ...
           -L2*s12  - L3*s123 - L4*s1234,          ...
           -L3*s123 - L4*s1234,                   ...
           -L4*s1234;
           
            L1*c1   + L2*c12  + L3*c123 + L4*c1234,  ...
            L2*c12  + L3*c123 + L4*c1234,          ...
            L3*c123 + L4*c1234,                   ...
            L4*c1234;
           
            1, 1, 1, 1 ];
end

%% ik_4R_jacobiana
function theta = ik_4R_jacobiana(xd, yd, phid, L)
    % ----------------------------------------------------------
    % IK iterativa 4R con Jacobiano (pseudoinversa).
    %   - inizializza theta_k
    %   - calcola errore e = x_d - f(theta_k)
    %   - aggiornamento: theta_k = theta_k + pinv(J)*e
    % ----------------------------------------------------------

    % Parametri
    maxIter = 100;
    tol = 1e-5;
    alpha = 0.5;

    % Inizializzazione
    persistent theta_k
    if isempty(theta_k)
        theta_k = [0 0 0 0]';   % 4x1
    end

    for k = 1:maxIter
        % Cinematica diretta
        [~, x, y, phi] = fk_4R_DH(theta_k', L);

        % errore dell'organo terminale (x,y,phi)
        e = [xd - x;
             yd - y;
             wrapToPi(phid - phi)];

        % condizione di arresto
        if norm(e) < tol
            break;
        end

        % Jacobiano (3x4)
        J = jacobian_4R(theta_k, L);

        % pseudo inversa -> (4x3) * (3x1) = 4x1
        dtheta = alpha * pinv(J) * e;

        % aggiornamento
        theta_k = theta_k + dtheta;
    end

    theta = theta_k;  % 4x1 (coerente con il tuo 3R)
end

%% Dinamica - Matrice di inerzia M
function M = M_3R(theta, L, m, l, I)
    t2 = theta(2); t3 = theta(3);
    L1=L(1); L2=L(2);
    l1=l(1); l2=l(2); l3=l(3);
    m1=m(1); m2=m(2); m3=m(3);
    I1=I(1); I2=I(2); I3=I(3);
    
    M11 = I1 + I2 + I3 ...
        + m1*l1^2 ...
        + m2*(L1^2 + l2^2 + 2*L1*l2*cos(t2)) ...
        + m3*(L1^2 + L2^2 + l3^2 ...
              + 2*L1*L2*cos(t2) ...
              + 2*L1*l3*cos(t2+t3) ...
              + 2*L2*l3*cos(t3));
    
    M22 = I2 + I3 ...
        + m2*l2^2 ...
        + m3*(L2^2 + l3^2 + 2*L2*l3*cos(t3));
    
    M33 = I3 + m3*l3^2;
    
    M12 = I2 + I3 ...
        + m2*(l2^2 + L1*l2*cos(t2)) ...
        + m3*(L2^2 + l3^2 ...
              + L1*L2*cos(t2) ...
              + L1*l3*cos(t2+t3) ...
              + L2*l3*cos(t3));
    
    M13 = I3 + m3*(l3^2 ...
                   + L1*l3*cos(t2+t3) ...
                   + L2*l3*cos(t3));
    
    M23 = I3 + m3*(l3^2 + L2*l3*cos(t3));
    
    M = [M11 M12 M13;
         M12 M22 M23;
         M13 M23 M33];
end

function M = M_4R_from_3R(theta, L, m, l, I)
    t2 = theta(2); t3 = theta(3); t4 = theta(4);

    L1=L(1); L2=L(2); L3=L(3);
    l4=l(4);
    m4=m(4); I4=I(4);

    % embed M_3R in 4x4
    M = zeros(4,4,'like',theta);
    M3 = M_3R(theta(1:3), L(1:3), m(1:3), l(1:3), I(1:3));
    M(1:3,1:3) = M3;

    % contributo link4
    r4r4 = l4^2;

    r3r4 = l4^2 + L3*l4*cos(t4);
    r2r4 = l4^2 + L3*l4*cos(t4) + L2*l4*cos(t3+t4);
    r1r4 = l4^2 + L3*l4*cos(t4) + L2*l4*cos(t3+t4) + L1*l4*cos(t2+t3+t4);

    r3r3 = L3^2 + l4^2 + 2*L3*l4*cos(t4);
    r2r3 = r3r3 + (L2*L3*cos(t3) + L2*l4*cos(t3+t4));
    r2r2 = L2^2 + r3r3 + 2*(L2*L3*cos(t3) + L2*l4*cos(t3+t4));

    r1r3 = r2r3 + L1*L3*cos(t2+t3) + L1*l4*cos(t2+t3+t4);
    r1r2 = r2r2 + (L1*L2*cos(t2) + L1*L3*cos(t2+t3) + L1*l4*cos(t2+t3+t4));
    r1r1 = L1^2 + r2r2 + 2*(L1*L2*cos(t2) + L1*L3*cos(t2+t3) + L1*l4*cos(t2+t3+t4));

    add = @(rij) (m4*rij + I4);

    % aggiorno blocco 4x4
    M11 = M(1,1) + add(r1r1);
    M22 = M(2,2) + add(r2r2);
    M33 = M(3,3) + add(r3r3);
    M44 = add(r4r4);

    M12 = M(1,2) + add(r1r2);
    M13 = M(1,3) + add(r1r3);
    M14 = add(r1r4);

    M23 = M(2,3) + add(r2r3);
    M24 = add(r2r4);

    M34 = add(r3r4);

    M = [M11 M12 M13 M14;
         M12 M22 M23 M24;
         M13 M23 M33 M34;
         M14 M24 M34 M44];
end

function C_fun = C4R_fun()
    % --- simboli ---
    syms t1 t2 t3 t4 real
    syms dt1 dt2 dt3 dt4 real

    syms L1 L2 L3 L4 real
    syms m1 m2 m3 m4 real
    syms l1 l2 l3 l4 real
    syms I1 I2 I3 I4 real

    theta  = [t1; t2; t3; t4];
    dtheta = [dt1; dt2; dt3; dt4];

    L = [L1; L2; L3; L4];
    m = [m1; m2; m3; m4];
    l = [l1; l2; l3; l4];
    I = [I1; I2; I3; I4];

    % --- M simbolica ---
 
    M = M_4R_from_3R(theta, L, m, l, I);  

    % --- Christoffel: C(theta,dtheta) ---
    n = 4;
    C = sym(zeros(n,n));
    for i = 1:n
        for j = 1:n
            cij = sym(0);
            for k = 1:n
                cij = cij + sym(1)/2 * ( ...
                    diff(M(i,j), theta(k)) + ...
                    diff(M(i,k), theta(j)) - ...
                    diff(M(j,k), theta(i)) ) * dtheta(k);
            end
            C(i,j) = simplify(cij);
        end
    end
    C = simplify(C, 'Steps', 50);

    % --- crea funzione numerica ---
    C_fun = matlabFunction(C, 'Vars', {theta, dtheta, L, m, l, I});
end

%% --- Funzione G ---
function G = g_4R(theta, L, m, l, g)
    t1 = theta(1); t2 = theta(2); t3 = theta(3); t4 = theta(4);

    L1 = L(1); L2 = L(2); L3 = L(3);
    l1 = l(1); l2 = l(2); l3 = l(3); l4 = l(4);

    m1 = m(1); m2 = m(2); m3 = m(3); m4 = m(4);

    a1 = t1;
    a2 = a1 + t2;
    a3 = a2 + t3;
    a4 = a3 + t4;
 

    % -------------------------------------------------
    % G1: contributo di tutti i link (1,2,3,4)
    % -------------------------------------------------
    G1 = g*( ...
          m1* l1 * cos(a1) ...
        + m2*(L1 * cos(a1) + l2 * cos(a2)) ...
        + m3*(L1 * cos(a1) + L2 * cos(a2) + l3 * cos(a3)) ...
        + m4*(L1 * cos(a1) + L2 * cos(a2) + L3 * cos(a3) ...
            + l4 * cos(a4)) );

    % -------------------------------------------------
    % G2: contributo link 2,3,4
    % -------------------------------------------------
    G2 = g*( ...
          m2* l2 * cos(a2) ...
        + m3*(L2 * cos(a2) + l3*cos(a3)) ...
        + m4*(L2 * cos(a2) + L3*cos(a3) ...
            + l4*cos(a4)) );

    % -------------------------------------------------
    % G3: contributo link 3,4
    % -------------------------------------------------
    G3 = g*( ...
          m3* l3 * cos(a3) ...
        + m4*(L3 * cos(a3) + l4 * cos(a4)) );

    % -------------------------------------------------
    % G4: contributo solo del link 4
    % -------------------------------------------------
    G4 = g*( m4 * l4 * cos(a4) );

    G = [G1; G2; G3; G4];
end

function [M, C, G] = dynamics_4R_2(theta, dtheta, params)
    persistent C_fun
    if isempty(C_fun)
        C_fun = C4R_fun();  % calcolo simbolico UNA VOLTA
    end

    L = params.L; m = params.m; l = params.l; I = params.I; g = params.g;

    M = M_4R_from_3R(theta, L, m, l, I);
    G = g_4R(theta, L, m, l, g);

    % Cmat e vettore h = Cmat*dtheta
    Cmat = C_fun(theta(:), dtheta(:), L(:), m(:), l(:), I(:));
    C = Cmat * dtheta(:);
end

function [theta_best, all_solutions] = ik_4R(x, y, phi, L, q_seed)

    if nargin < 5
        q_seed = [];
    end

    persistent q_prev;
    if isempty(q_prev)
        q_prev = zeros(4,1);
    end
    if isempty(q_seed)
        q_seed = q_prev;
    else
        q_seed = q_seed(:);
    end

    L1=L(1); L2=L(2); L3=L(3); L4=L(4);

    % Polso (a monte del link 4)
    wx = x - L4*cos(phi);
    wy = y - L4*sin(phi);

    N = 61;
    theta4_grid = linspace(-pi, pi, N);

    sols = [];
    costs = [];

    for t4 = theta4_grid
        phi3 = wrapToPi(phi - t4);

        px = wx - L3*cos(phi3);
        py = wy - L3*sin(phi3);

        D = (px^2 + py^2 - L1^2 - L2^2)/(2*L1*L2);
        if abs(D) > 1
            continue; % non raggiungibile per questo t4
        end

        for elbow = [+1, -1]
            s2 = elbow * sqrt(max(0,1-D^2));
            c2 = D;
            t2 = atan2(s2, c2);
            t1 = atan2(py, px) - atan2(L2*sin(t2), L1 + L2*cos(t2));

            % chiudo t3 per rispettare phi3
            t3 = wrapToPi(phi3 - (t1 + t2));

            % theta4 è il parametro scelto
            t4c = wrapToPi(t4);

            th = [wrapToPi(t1); wrapToPi(t2); wrapToPi(t3); t4c];

            % Costo: vicinanza al seed (continuità)
            d = wrapToPi(th - q_seed);
            cost = d.'*d;

            sols = [sols, th];
            costs = [costs, cost];
        end
    end

    all_solutions = sols;

    [~, idx] = min(costs);
    theta_best = sols(:, idx);

    q_prev = theta_best;
end

function dx = dyn4R_ode(t, x, params, traj, gains, CONTROL_MODE)
    % Stato
    q  = x(1:4);
    dq = x(5:8);

    % End-effector (serve per TASK mode)
    [~, xe, ye, ~] = fk_4R_DH(q.', params.L);

    % Controllo: tau
    tau = control_tau_4R(t, q, dq, params, traj, gains, CONTROL_MODE, xe, ye);

    % Dinamica
    [M, C, G] = dynamics_4R_2(q, dq, params);
    ddq = M \ (tau - C - G);

    dx = [dq; ddq];
end


function [tau, e_q, e_x] = control_tau_4R(t, q, dq, params, traj, gains, CONTROL_MODE, xe, ye)
    % Default output errori
    e_q = zeros(4,1);
    e_x = zeros(2,1);

    % Interpolazione desiderati a tempo t (coerente con ode45)
    qd  = interp1(traj.t, traj.qd.',  t, 'linear', 'extrap').';
    xd  = interp1(traj.t, traj.xd.',  t, 'linear', 'extrap').';
    dxd = interp1(traj.t, traj.dxd.', t, 'linear', 'extrap').';

    % Dinamica per compensazione gravità
    [~, ~, G] = dynamics_4R_2(q, dq, params);

    if CONTROL_MODE == "JOINT"
        dqd = zeros(4,1);

        e_q  = qd - q;
        ed_q = dqd - dq;

        tau = gains.Kp*e_q + gains.Kd*ed_q + G;

    else
        % TASK: controllo su (x,y)
        J = jacobian_4R(q, params.L);
        Jxy = J(1:2,:);

        e_x  = xd - [xe; ye];
        dx_now = Jxy*dq;
        de_x = dxd - dx_now;

        F = gains.Kp_t*e_x + gains.Kd_t*de_x;

        tau = Jxy.' * F + G;
    end
end
