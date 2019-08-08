[x,y,z]=peaks(100);

       a1 =      1
       a2 =      1 
       b1 =     1 
       b2 =     0
       c1 =      0.5
       c2 =       0.1
% z=a1.*exp(-((x-b1)/c1).^2)*a2.*exp(-((y-b2)/c2).^2)+a1.*exp(-((x-1)/c1).^2)*a2.*exp(-((y-1)/c2).^2);
z=a1.*exp(-(((x-b1).*cos(pi/3)-(y-b2).*sin(pi/3))/c1).^2)*a2.*exp(-(((y-b2).*cos(pi/3)+(x-b1).*sin(pi/3))/c2).^2);

theta=pi/3;
car_x=2;
car_y=1;
car_weight=0.3;
car_height=0.5;
z=exp(-(((x-car_x).*cos(theta)-(y-car_y).*sin(theta))/car_weight).^2).* ...
    exp(-(((y-car_y).*cos(theta)+(x-car_x).*sin(theta))/car_height).^2);

theta=-pi/3;
car_x=-2;
car_y=0;
car_weight=0.3;
car_height=0.5;
z=z+exp(-(((x-car_x).*cos(theta)-(y-car_y).*sin(theta))/car_weight).^2).* ...
    exp(-(((y-car_y).*cos(theta)+(x-car_x).*sin(theta))/car_height).^2);
surf(x, y, z)