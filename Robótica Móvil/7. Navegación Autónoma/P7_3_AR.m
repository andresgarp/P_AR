function P7_3_AR(Nstart,Nend)
%P5_AR Práctica 5: Navegación Autónoma
%   Se calculará la trayectoria optima del nodo inicial "Nstart" al final
%   "Nend" mediante Dijkstra, y se desplazará el robot por el mapa con
%   evasión de obstáculos mediante campos potenciales.

clc
close all

% Cargamos el mapa topológico
mapa3 % De aquí obtenemos "nodos" y "costes"

% Cargamos el mapa
map_img = imread('mapa2.pgm');
map_neg = imcomplement(map_img);
map_bin = imbinarize(map_neg);
mapa = binaryOccupancyMap(map_bin);
show(mapa);
hold on;

% Dibujamos el origen y el destino
plot(nodos(Nstart,2), nodos(Nstart,3), 'go','MarkerFaceColor','green');
plot(nodos(Nend,2), nodos(Nend,3), 'ro','MarkerFaceColor','red');

% Calculamos la Heurística con la distancia euclídea
heur = zeros(length(nodos));
for i = 1:1:length(nodos)
    for j = 1:1:length(nodos)
        Pi = [nodos(i, 2), nodos(i, 3)];
        Pj = [nodos(j, 2), nodos(j, 3)];
        heur(i,j) = norm(Pi-Pj);
    end
end

% Aprovechamos la función de dijkstra previa
[coste, ruta] = aestrella(costes, heur, Nstart, Nend);

fprintf("Ruta calculada. Coste:"); disp(coste);

% Configuracion del sensor (laser de barrido)
max_rango = 10;
angulos = -pi/2:(pi/180):pi/2; % resolucion angular barrido laser

% Caracteristicas del vehiculo y parametros del metodo
v = 0.4;         % Velocidad del robot
D = 1.5;         % Rango del efecto del campo de repulsión de los obstáculos
alpha = 1;        % Coeficiente de la componente de atracción
beta = 1000;      % Coeficiente de la componente de repulsión

%% Inicialización

robot = [nodos(Nstart,2), nodos(Nstart,3) 0];   % El robot empieza en la posición de origen (orientacion cero)
path = [];            % Se almacena el camino recorrido
path = [path; robot]; % Se añade al camino la posicion actual del robot
iteracion = 0;        % Se controla el nº de iteraciones por si se entra en un minimo local

%% Calculo de la trayectoria

for i = 1:1:length(ruta)
    destino = [nodos(ruta(i), 2), nodos(ruta(i), 3)];
    while norm(destino-robot(1:2)) > v && iteracion<1000    % Hasta menos de una iteración de la meta (10 cm)
       
       qRobot = robot(:, 1:2);
       % La fuerza de atracción
       Fa = alpha * (destino - qRobot);
    
       % Las fuerzas de repulsión
       obst = SimulaLidar(robot, mapa, angulos, max_rango);
       Fr = [0 0];
       for j = 1:1:length(obst)
           if ~isnan(obst(j))
               dist = norm(qRobot - obst(j, :));
               if (dist <= D)
                   Fr = Fr + beta*(1/dist - 1/D)*(qRobot - obst(j, :))/dist;
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
       plot(path(:,1),path(:,2),'r');

       drawnow
    
       iteracion = iteracion+1;
    end

    if iteracion == 1000   % Se ha caído en un mínimo local
        fprintf('No se ha podido llegar al destino.\n')
        break
    end
end

if iteracion < 1000
    fprintf('Destino alcanzado.\n')
end

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