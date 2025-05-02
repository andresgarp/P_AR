function w = curva(x, y, theta, xo, yo, v)
% Calculo de la velocidad angular para hacer una curva tangente al eje x
% del robot hasta el punto objetivo.
% x y: Posición del robot
% Theta: Orientación del robot (en radianes)
% xo yo: Punto objetivo
% v: Velocidad lineal
    
    % Ecuación de la recta perpendicular a la dirección del robot
    % Pasa por (x, y) y tiene pendiente -1/m
    m_perp = -1/tan(theta);
    b_perp = y - m_perp * x;
    
    % Mediatriz del segmento que une (x, y) y (xo, yo)
    x_mid = (x + xo) / 2;
    y_mid = (y + yo) / 2;

    slope_segment = (yo - y)/(xo - x);
    % Evitar división por cero para segmentos verticales
    if isinf(slope_segment)
        m_mediatriz = 0;
        b_mediatriz = y_mid;
    elseif slope_segment == 0
        m_mediatriz = Inf;
        b_mediatriz = x_mid;
    else
        m_mediatriz = -1/slope_segment;
        b_mediatriz = y_mid - m_mediatriz*x_mid;
    end
    
    % Resolver sistema para encontrar el centro (xc,yc)
    if isinf(m_mediatriz)
        % Caso mediatriz vertical
        xc = x_mid;
        yc = m_perp*xc + b_perp;
    elseif m_perp == m_mediatriz
        % Rectas paralelas - solución especial
        w = 0;
        return;
    else
        % Sistema general
        xc = (b_mediatriz - b_perp)/(m_perp - m_mediatriz);
        yc = m_perp*xc + b_perp;
    end

    % Radio de la circunferencia
    R = sqrt((xc - x)^2 + (yc - y)^2);
        
    % Vemos si está a la izquierda (1) o a la derecha (-1)
    signo = sign(cos(theta)*(yo-y)-sin(theta)*(xo-x));
    
    % Velocidad angular del robot para satisfacer la circunferencia de
    % dicho radio a 0.3m/s
    w = signo*v/R; 
end