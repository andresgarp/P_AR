% Parámetros del sistema
k = 1;
tau = 0.12;
l = 0.4;
R = 0.1;

% Datos de entrada
w_id = 15; %Velocidad deseada de la rueda izquierda
w_dd = 15; % " de la derecha
tmax = 10; % Tiempo máximo de simulación
t_inc = 0.025; % Tiempo de muestreo

% Inicializamos
x = 0;
y = 0;
theta = 0;
wi = zeros(1, tmax/t_inc);
wd = zeros(1, tmax/t_inc);
i = 1;
figure

for t = 0:t_inc:tmax 
    if t > 9
        w_id = 10;
        w_dd = -10;
    elseif t > 5 
        w_id = 1;
        w_dd = 10;
    elseif t > 3
        w_id = 10;
        w_dd = 1;
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
    hold on

end
hold off

% Dibujar las velocidades para ver la evolución
tiempo = 0:t_inc:tmax;
figure
plot(tiempo, wi(1:length(tiempo)))
hold on
plot(0:t_inc:tmax, wd(1:length(tiempo)))
hold off
