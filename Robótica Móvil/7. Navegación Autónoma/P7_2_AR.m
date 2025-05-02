function P7_2_AR(Nstart,Nend)
%P5_AR Práctica 5: Navegación Autónoma
%   Se calculará la trayectoria optima del nodo inicial "Nstart" al final
%   "Nend" mediante Dijkstra, y se desplazará el robot por el mapa con
%   evasión de obstáculos mediante navegación reactiva

clc
close all

% Cargamos el mapa topológico
mapa2 % De aquí obtenemos "nodos" y "costes"

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

% Aprovechamos la función de dijkstra previa
[coste, ruta] = dijkstra(costes, Nstart, Nend);

fprintf("Ruta calculada. Coste:"); disp(coste);

% Configuracion del sensor (laser de barrido)
max_rango = 10;
angulos = -pi/2:(pi/180):pi/2; % resolucion angular barrido laser

% Caracteristicas del vehiculo y parametros del metodo
v = 0.4;         % Velocidad del robot
D = 1.5;         % Rango del efecto del campo de repulsión de los obstáculos
w = 60*pi/180;   % Velocidad angular del giro al evitar obstáculos

%% Inicialización

robot = [nodos(Nstart,2), nodos(Nstart,3) 0];   % El robot empieza en la posición de origen (orientacion cero)
path = [];            % Se almacena el camino recorrido
path = [path; robot]; % Se añade al camino la posicion actual del robot
iteracion = 0;        % Se controla el nº de iteraciones por si se entra en un minimo local
alpha = 0;

%% Calculo de la trayectoria

for i = 1:1:length(ruta)
    destino = [nodos(ruta(i), 2), nodos(ruta(i), 3)];
    while norm(destino-robot(1:2)) > v && iteracion<1000    % Hasta menos de una iteración de la meta (10 cm)
       
       qRobot = robot(:, 1:2);
   
       obst = SimulaLidar(robot, mapa, angulos, max_rango);
       
       % Vemos si hay un objeto cerca y el ángulo que este tiene
       dist_min = D;
       ang = NaN;
       for j = 1:1:length(obst)
           if ~isnan(obst(j))
               dist = norm(qRobot - obst(j, :));
               angulo = atan2(obst(j,2)-qRobot(2), obst(j,1)-qRobot(1)) - robot(:, 3);

               angulo = wrapToPi(angulo);
               
               if abs(angulo) < 0.6
                   dist2 = dist/1.5;
               else
                   dist2 = dist;
               end

               if dist2 < dist_min
                   dist_min = dist2;
                   ang = angulo;
               end
               
           end 
       end

       % Calculamos el ángulo de giro alpha en función del angulo donde
       % está el obstáculo
       if abs(alpha) > 0.01 && abs(ang) < 90*pi/180
           % Cambiamos alpha por 30
           alpha = -sign(ang)*30*pi/180;
       else
           if abs(ang) <= 30*pi/180
               alpha = -sign(ang)*60*pi/180;
           elseif abs(ang) <= 50*pi/180
               alpha = -sign(ang)*30*pi/180;
           else
               if dist_min < D && ~isnan(ang)
                   alpha = -sign(ang)*30*pi/180;
               else
                   alpha = 0;
               end
           end
       end

       previous = robot(1:2);
       switch abs(alpha)
           case 0
               Ft = destino - qRobot;
               Ftn = Ft/norm(Ft);
               % Desplazamos al robot
               robot(1:2) = robot(1:2) + v*Ftn;
               % Actualizamos su ángulo
               robot(3) = atan2(Ftn(2), Ftn(1));
           case 30*pi/180
               d = abs(2*(v/w)*sin(alpha/2));
               robot(1) = robot(1) + d*cos(robot(3)+alpha/2) + v/2*cos(robot(3)+alpha);
               robot(2) = robot(2) + d*sin(robot(3)+alpha/2) + v/2*sin(robot(3)+alpha);
               robot(3) = robot(3) + alpha;
           case 60*pi/180
               d = abs(2*(v/w)*sin(alpha/2));
               robot(1) = robot(1) + d*cos(robot(3) + alpha/2);
               robot(2) = robot(2) + d*sin(robot(3) + alpha/2);
               robot(3) = robot(3) + alpha;
       end       
    
       % Se añade la nueva posición al camino seguido
       path = [path; robot];
       plot(path(:,1),path(:,2),'r');
       %quiver(robot(1), robot(2), cos(robot(3)), sin(robot(3)))

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