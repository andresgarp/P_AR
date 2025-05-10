 %Calculo del RMSE:
%Cálculo de la distancia para el error cuadrática de distancia 
e_dist=0; %Varaible donde guardaremos el valor del error de distancia 
e_o=0;
for i=1:data.i
    e_dist=e_dist+sqrt( (data.true(1,i)-data.path(1,i))^2 + (data.true(2,i)-data.path(2,i))^2 );
    e_o=e_o+(data.true(3,i)-data.path(3,i));
end
RMSE_dist=sqrt((e_dist^2)/data.i)
RMSE_o=sqrt((e_o)^2/data.i)
