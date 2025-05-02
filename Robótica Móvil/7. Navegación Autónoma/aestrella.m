function [cost, route] = aestrella(G, H, start, finish)
%AESTRELLA Find the optimal path from start to finish of G considering
%heurístic H


% El número de nodos
n = size(G,1);

% Inicializamos el vector temporal con todos los nodos
temporal = Inf(n,4);
temporal(start, :) = [0,0,0,0];
temporal(:,1) = (1:n)';
definitivo = [];

%Extraemos el nodo inicial
temporal = sortrows(temporal, 3);
nodo = 0;

% Bucle hasta llegar al nodo final
while nodo ~= finish
    nodo = temporal(1,1);
    % Recorremos el nodo de los que aún no se han analizado
    for i = 2:1:size(temporal)
        if G(nodo, temporal(i,1)) > 0
            % Distancia para llegar a ese nodo
            g = temporal(1,2) + G(nodo, temporal(i,1));
            
            % Coste estimado
            f = g + H(finish, temporal(i,1));
            
            
            % Si la distancia es menor sobreescribirla
            if (g < temporal(i,2))
                temporal(i,2) = g;
                temporal(i,3) = f;
                temporal(i,4) = nodo;
            end
        end
    end

    % Actualizamos los vectores temporales y definitivos
    definitivo = [definitivo; temporal(1,:)];
    temporal = temporal(2:end, :);
    temporal = sortrows(temporal, 3);
end

cost = definitivo(end, 2);
if cost == Inf
    route = [];
    return
end

% Sacamos la ruta recorriendo el camino a la inveresa
route = [definitivo(end, 1)];
previo = definitivo(end, 4);
while previo ~= 0
    % Buscamos el nodo predecesor en la matriz
    nodo = find(definitivo(:, 1) == previo, 1);
    
    % Actualizamos la ruta
    route = [previo, route];

    previo = definitivo(nodo, 4);
end

end
