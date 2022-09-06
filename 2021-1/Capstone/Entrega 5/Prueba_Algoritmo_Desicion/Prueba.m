%%calcular extremos del obstaculo
obs0=[550 0];
largo0=100.0;
largo1=150.0;
angle=90.0;
%%extremo 1
aux1=obs0(1)+largo0*cos(angle*pi/180);
aux2=obs0(2)+largo0*sin(angle*pi/180);
obs1=[aux1 aux2];
%%extremo 2
aux1=obs0(1)-largo1*cos(angle*pi/180);
aux2=obs0(2)-largo1*sin(angle*pi/180);
obs2=[aux1 aux2];

%%agregar puntos entremedio
i=1;
obs_1_0=[];
obs_1_1=[];
while  ((i*10.0)<=largo0)
    aux1=obs1(1)-i*10.0*cos(angle*pi/180);
    aux2=obs1(2)-i*10.0*sin(angle*pi/180);
    obs_1_0=[obs_1_0 aux1];
    obs_1_1=[obs_1_1 aux2];
    i=i+1;
end

i=1;
obs_2_0=[];
obs_2_1=[];
while  ((i*10.0)<=largo1)
    aux1=obs2(1)+i*10.0*cos(angle*pi/180);
    aux2=obs2(2)+i*10.0*sin(angle*pi/180);
    obs_2_0=[obs_2_0 aux1];
    obs_2_1=[obs_2_1 aux2];
    i=i+1;
end

%%lista de puntos del obstaculo
x=[obs0(1) obs_1_0 obs1(1) obs_2_0 obs2(1)];
y=[obs0(2) obs_1_1 obs1(2) obs_2_1 obs2(2)];

%scatter(x,y,'filled')
plot(x,y)
hold on

%%vehiculo
pos_x=400;
pos_y=0;
l_1=obs0(1)-pos_x;
q_1=0;
l_2=sqrt((obs1(1)-pos_x)^2+(obs1(2)-pos_y)^2);
q_2=atan((obs1(2)-pos_y)/(obs1(1)-pos_x));

posiciones_x=[200,300, 300.3185306842933, 473, 647, 821, 995, 1000, 1000, 1000, 1000, 1000, 800, 600, 400, 200, 0, 0, 0, 0, 0, 0];
posiciones_y=[0, 0,399.9998731727338, 299, 200, 101, 2, 200, 400, 600, 800, 1000, 1000, 1000, 1000, 1000, 1000, 800, 600, 400, 200, 0];

posx=[200, 1000, 1000, 1000, 1000, 1000, 800, 600, 400, 200, 0, 0, 0, 0, 0, 0];
posy=[0, 200, 400, 600, 800, 1000, 1000, 1000, 1000, 1000, 1000, 800, 600, 400, 200, 0];
scatter(posiciones_x,posiciones_y,'filled')

scatter(posx,posy,'filled')

hold off


