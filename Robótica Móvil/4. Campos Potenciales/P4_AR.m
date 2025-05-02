% AMPLIACION DE ROBOTICA
% PRACTICA 4: Navegacion local con campos potenciales
% Evitar obstaculos

clc
clearvars
close all
%% Carga del mapa de ocupacion

map_img = imread('mapa1_150.png');
map_neg = imcomplement(map_img);
map_bin = imbinarize(map_neg);
mapa = binaryOccupancyMap(map_bin);
show(mapa);

% Marcar los puntos de inicio y destino
hold on;
title('Señala los puntos inicial y final de la trayectoria del robot');
origen = ginput(1);
plot(origen(1), origen(2), 'go','MarkerFaceColor','green');  % Dibujamos el origen
destino = ginput(1);
plot(destino(1), destino(2), 'ro','MarkerFaceColor','red');  % Dibujamos el destino

% Configuracion del sensor (laser de barrido)
max_rango = 10;
angulos = -pi/2:(pi/180):pi/2; % resolucion angular barrido laser

% Caracteristicas del vehiculo y parametros del metodo
v = 0.4;         % Velocidad del robot
D = 1.5;         % Rango del efecto del campo de repulsión de los obstáculos
alpha = 1;        % Coeficiente de la componente de atracción
beta = 150;      % Coeficiente de la componente de repulsión

%% Inicialización

robot = [origen 0];   % El robot empieza en la posición de origen (orientacion cero)
path = [];            % Se almacena el camino recorrido
path = [path; robot]; % Se añade al camino la posicion actual del robot
iteracion = 0;        % Se controla el nº de iteraciones por si se entra en un minimo local

%% Calculo de la trayectoria

while norm(destino-robot(1:2)) > v && iteracion<1000    % Hasta menos de una iteración de la meta (10 cm)
   qRobot = robot(:, 1:2);
   % La fuerza de atracción
   Fa = alpha * (destino - qRobot);

   % Las fuerzas de repulsión
   obst = SimulaLidar(robot, mapa, angulos, max_rango);
   Fr = [0 0];
   for i = 1:1:length(obst)
       if ~isnan(obst(i))
           dist = norm(qRobot - obst(i, :));
           if (dist <= D)
               Fr = Fr + beta*(1/dist - 1/D)*(qRobot - obst(i, :))/dist;
           end   
       end
   end

   % Sumamos todas las fuerzas para obtener el total
   Ft = Fa + Fr;
   Ftn = Ft/norm(Ft); % Calculamos su normal

   % Desplazamos al robot
   robot(1:2) = robot(1:2) + v*Ftn;
   % Actualizamos su ángulo
   robot(3) = atan2(Ftn(2), Ftn(1));

   % Se añade la nueva posición al camino seguido
   path = [path; robot];
    
    % Dibujo de las fuerzas de Manu
    cla;  % Limpia la figura antes de dibujar de nuevo
    show(mapa); hold on;  % Redibuja el mapaNo,
    plot(origen(1), origen(2), 'go','MarkerFaceColor','green');  % Dibujamos el origen
    plot(destino(1), destino(2), 'ro','MarkerFaceColor','red');  % Dibujamos el destino
    plot(path(:,1),path(:,2),'r');  % Vuelve a dibujar el path
    % Representar F_rep en azul desde la posición actual del robot
    quiver(robot(1), robot(2), Fr(1), Fr(2), 0.5, 'b', 'LineWidth', 1.5);
    % Representar F_atr en amarillo desde la posición actual del robot
    quiver(robot(1), robot(2), Fa(1), Fa(2), 0.5, 'y', 'LineWidth', 1.5);
    % Representar F_final en magenta (opcional)
    quiver(robot(1), robot(2), Ft(1), Ft(2), 0.5, 'm', 'LineWidth', 1.5);

   %plot(path(:,1),path(:,2),'r');
   drawnow

   iteracion = iteracion+1;
end

if iteracion == 1000   % Se ha caído en un mínimo local
    fprintf('No se ha podido llegar al destino.\n')
else
    fprintf('Destino alcanzado.\n')
end

%% funcion para simular el sensor
function [obs] = SimulaLidar(robot, mapa, angulos, max_rango)
    obs=rayIntersection(mapa,robot,angulos, max_rango);
    % plot(obs(:,1),obs(:,2),'*r') % Puntos de interseccion lidar
    % plot(robot(1),robot(2),'ob') % Posicion del robot
    % for i = 1:length(angulos)
    %     plot([robot(1),obs(i,1)],...
    %         [robot(2),obs(i,2)],'-b') % Rayos de interseccion
    % end
    % % plot([robot(1),robot(1)-6*sin(angulos(4))],...
    % %     [robot(2),robot(2)+6*cos(angulos(4))],'-b') % Rayos fuera de
    % %     rango
end