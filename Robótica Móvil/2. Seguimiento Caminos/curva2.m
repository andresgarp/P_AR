function w = curva2(x, y, theta, xo, yo, v)
% Calcula la velocidad angular para una trayectoria circular tangente a la
% orientación del robot y que pasa por ambos puntos: (x,y) y (xo,yo)
    
    % Vector dirección del robot
    dir_vec = [cos(theta); sin(theta)];
    
    % Vector al punto objetivo
    target_vec = [xo - x; yo - y];
    
    % Producto cruzado para determinar el sentido de giro
    cross_prod = dir_vec(1)*target_vec(2) - dir_vec(2)*target_vec(1);
    
    % 1. Recta perpendicular a la dirección del robot que pasa por (x,y)
    m_perp = -1/tan(theta);
    b_perp = y - m_perp*x;
    
    % 2. Mediatriz del segmento (x,y)-(xo,yo)
    mid_point = [(x + xo)/2; (y + yo)/2];
    slope_segment = (yo - y)/(xo - x);
    
    % Evitar división por cero para segmentos verticales
    if isinf(slope_segment)
        m_mediatriz = 0;
        b_mediatriz = mid_point(2);
    elseif slope_segment == 0
        m_mediatriz = Inf;
        b_mediatriz = mid_point(1);
    else
        m_mediatriz = -1/slope_segment;
        b_mediatriz = mid_point(2) - m_mediatriz*mid_point(1);
    end
    
    % Resolver sistema para encontrar el centro (xc,yc)
    if isinf(m_mediatriz)
        % Caso mediatriz vertical
        xc = mid_point(1);
        yc = m_perp*xc + b_perp;
    elseif m_perp == m_mediatriz
        % Rectas paralelas - solución especial
        w = 0;
        return;
    else
        % Sistema general
        A = [1, -m_perp; 1, -m_mediatriz];
        B = [b_perp; b_mediatriz];
        xc = (B(2) - B(1))/(m_perp - m_mediatriz);
        yc = m_perp*xc + b_perp;
    end
    
    % Calcular radio
    R = sqrt((xc - x)^2 + (yc - y)^2);
    
    % Verificación de que pasa por ambos puntos
    R_o = sqrt((xc - xo)^2 + (yc - yo)^2);
    if abs(R - R_o) > 1e-6
        disp('La circunferencia no pasa por el punto objetivo');
    end
    
    % Calcular velocidad angular con signo correcto
    w = sign(cross_prod)*v/R;
    
   
end