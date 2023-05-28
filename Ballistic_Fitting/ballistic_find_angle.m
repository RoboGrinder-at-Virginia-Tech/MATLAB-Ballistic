function [angle, is_valid] = ballistic_find_angle(initial_vel, tar_dist)

%initial_vel=15;
air_den = 1.293;
proj_area = .02^2*pi;
cd = (0.47*air_den*proj_area)/2;
mass = 0.041;
y_dot =@(t,y)([y(3); y(4); -cd*sqrt(y(3)^2+y(4)^2)*y(3); -9.8-cd*sqrt(y(3)^2+y(4)^2)./mass*y(4)]); %insert function to be solved

target = [tar_dist;0];
err_tor = 1e-4;
finish = false;
iter = 0;
angle_h = 45*pi/180;
angle_l = 0*pi/180;


while ~finish
    angle = (angle_h+angle_l)/2;
    h = 0.001;  % set the step size
    t = 0:h:1;  % set the interval of x
    y = zeros(4, length(t));
    y(:,1) = [0; 0; initial_vel*cos(angle); initial_vel*sin(angle)];   % set the intial value for y
    n = length(t)-1;

    for i = 1:n   % Runge-Kutta
        k1 = y_dot(t(i),y(:,i));
        k2 = y_dot(t(i)+.5*h,y(:,i)+.5*k1*h);
        k3 = y_dot(t(i)+.5*h,y(:,i)+.5*k2*h);
        k4 = y_dot(t(i)+h,y(:,i)+k3*h);
        y(:,i+1) = y(:,i)+((k1+2*k2+2*k3+k4)./6)*h;
        
    end
    iter = iter + 1;
    [d ,index] = min(abs(y(1,:)-target(1)));

    if(abs(y(2,index)-target(2)) < err_tor)
        finish = true;
        is_valid = true;
    elseif(iter>15)
        finish = true;
        is_valid = false;
    else
        if (y(2,index)-target(2)>0)
            angle_h = angle;
        else
            angle_l = angle;
        end
    end

end
end
