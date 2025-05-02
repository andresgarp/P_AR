% Parámetros del sistema
k = 1;
tau = 0.12;
l = 0.4;
R = 0.1;
G = 0.25; %Ganancia del control proporcional

% Datos de entrada
t_inc = 0.025; % Tiempo de muestreo
tmax = 150; % Tiempo máximo de simulación
camino = [0,0;
        20, 0;
        20, 20;
        -10, 30;
        -20, -10;
        0, -30;
        0, 0];
v_d = 1.2;

% Inicializamos
x = 0;
y = 0;
theta = 0;
wi = zeros(1, round(tmax/t_inc));
wd = zeros(1, round(tmax/t_inc));
i = 1;
obj = 1;
t = 0;
figure

while (obj < size(camino, 1)+1 && t < tmax)
    % Como el GPS nos proprociona información cada 0.3s, vemos si el tiempo
    % es múltiplo de este valor (con un margen para los errores de coma flotante)
    if (abs(mod(t, 0.3)) < 0.01 || abs(mod(t, 0.3)) > 0.029)
        %Leemos el GPS
        GPS = DGPS(x, y, theta);
        x_gps = GPS(1);
        y_gps = GPS(2);
        theta_gps = GPS(3);

        % Calculamos las coordenadas del objetivo desde el robot
        xo = cos(theta_gps)*((camino(obj,1)-x_gps)) + sin(theta_gps)*((camino(obj,2)-y_gps));
        yo = -sin(theta_gps)*((camino(obj,1)-x_gps)) + cos(theta_gps)*((camino(obj,2)-y_gps));
    
        % Comprobamos si la distancia al objetivo es <1m para pasar al siguiente
        dist = sqrt(xo^2 + yo^2);
        if (dist < 1)
            obj = obj+1;
        end
    
        % Calculamos el ángulo al objetivo y la velocidad angular
        phi = atan2(yo, xo);
        w_d = G*phi*v_d;
    
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
        
        i = i+1;
    
        % Calculamos la velocidad de cada rueda
        wi(i) = exp(-t_inc/tau)*wi(i-1) + (1-exp(-t_inc/tau))*w_id;
        wd(i) = exp(-t_inc/tau)*wd(i-1) + (1-exp(-t_inc/tau))*w_dd;
        
        % Calculamos la velocidad líneal y angular
        v = (wi(i) + wd(i))*R/2;
        w = (wd(i)-wi(i))*R/(2*l);
    end

    % Por último calculamos la odometría
    theta = theta + w*t_inc;
    x = x + cos(theta) * v*t_inc;
    y = y + sin(theta) * v*t_inc;

    % Dibujamos por pantalla
    plot(x, y, "*", "color","k")
    plot(x+cos(theta)*xo-sin(theta)*yo, y+sin(theta)*xo+cos(theta)*yo, "X", "color", "r")
    plot(x_gps, y_gps, "O", "color", "g")
    hold on

    t = t + t_inc;

end
%legend("GPS", "Odometry", "Objective")
hold off

% Dibujar las velocidades para ver la evolución
tiempo = 0:t_inc:tmax;
figure
plot(tiempo(1:length(wi)), wi)
hold on
plot(tiempo(1:length(wd)), wd)
legend("Left wheel", "Right wheel", "Linear velocity")
hold off
