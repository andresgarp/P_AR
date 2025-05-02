clear all
close all
clc

% Entorno cerrado definido por una lista de puntos
%x=[-10 -10 10 10 20 -10]';
%y=[-10 10 20  0 -10 -10]';
x=[0 30 30 0 0]';
y=[0 0 10 10 0]';

% Posición y orientación del vehiculo
x0= 25;
y0= 8;
phi0= 0; % Entre -pi y pi

rangos= laser2D(x, y, x0, y0, phi0);

dibujaBarrido(x, y, x0, y0, phi0, rangos);

figure

dibujaBarrido(x, y, 0, 0, 0, rangos);


