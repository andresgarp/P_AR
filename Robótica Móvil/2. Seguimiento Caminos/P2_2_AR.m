% Parámetros del sistema
k = 1;
tau = 0.12;
l = 0.4;
R = 0.1;

% Datos de entrada
t_inc = 0.025; % Tiempo de muestreo
tmax = 380; % Tiempo máximo de simulación
v_d = 0.3;
% Entorno cerrado definido por una lista de puntos
xe=[0 30 50 50 80 80 120 120 80 80 50 50 30 30 0 0]';
ye=[0 0 0 2 2 4 0 10 14 16 16 12 12 10 10 0]';

% Inicializamos
x = 5;
y = 2;
theta = pi/4;
wi = zeros(1, round(tmax/t_inc));
wd = zeros(1, round(tmax/t_inc));
i = 1;
t = 0;
figure
plot(xe, ye)
hold on

while (t < tmax)
    % Como el láser nos proprociona información cada 0.5s, vemos si el tiempo
    % es múltiplo de este valor (con un margen para los errores de coma flotante)
    if (abs(mod(t, 0.5)) < 0.01 || abs(mod(t, 0.5)) > 0.49)
        % Leemos el láser
        laser = laser2D(xe, ye, x, y, theta);
        
        % Extraemos la mínima distancia
        [dist, haz] = min(laser);
        
        if haz > 36 
            haz_op = haz-36;
        else
            haz_op = haz + 36;
        end

        dist_op = laser(haz_op);
        
        % Calculamos el ángulo al que está dicha mín distancia
        phi = haz*5*pi/180;
        phi_op = haz_op*5*pi/180;
            
        % Con la distancia y el ángulo calculamos las coordendas del muro
        xp = x + (dist)*cos(theta+phi);
        yp = y + (dist)*sin(theta+phi);
        xpo = x + (dist_op)*cos(theta+phi_op);
        ypo = y + (dist_op)*sin(theta+phi_op);
        plot(xp, yp, "X", "color", "r")
        plot(xpo, ypo, "X", "color", "b")

        xo = x + 1;
        yo = (yp + ypo)/2;

        % Calculamos la velocidad angular para llegar allí
        w_d = curva(x, y, theta, xo, yo, v_d);
        
    
        % Con las velocidades deseadas calculamos la velocidad de cada rueda
        % deseada
        w_id = (v_d - l*w_d)/R;
        w_dd = (v_d + l*w_d)/R;
    
        % Limitamos la velocidad de la rueda a 15rad/s
        if w_id > 15
            w_id = 15;
        end
        if w_dd > 15
            w_dd = 15;
        end
    end

    i = i+1;
    
    % Calculamos la velocidad de cada rueda
    wi(i) = exp(-t_inc/tau)*wi(i-1) + (1-exp(-t_inc/tau))*w_id;
    wd(i) = exp(-t_inc/tau)*wd(i-1) + (1-exp(-t_inc/tau))*w_dd;
        
    % Calculamos la velocidad líneal y angular
    v = (wi(i) + wd(i))*R/2;
    w = (wd(i)-wi(i))*R/(2*l);

    % Por último calculamos la odometría
    theta = theta + w*t_inc;
    x = x + cos(theta) * v*t_inc;
    y = y + sin(theta) * v*t_inc;

    % Dibujamos por pantalla
    plot(x, y, "*", "color","k")
    

    t = t + t_inc;

    title(sprintf('%.1f / %.1f', t, tmax));
    pause(t_inc)
end
%legend("GPS", "Odometry", "Objective")
hold off

% Dibujar las velocidades para ver la evolución
tiempo = 0:t_inc:tmax;
figure
plot(tiempo, wi(1:length(tiempo)))
hold on
plot(tiempo, wd(1:length(tiempo)))
legend("Left wheel", "Right wheel", "Linear velocity")
hold off
