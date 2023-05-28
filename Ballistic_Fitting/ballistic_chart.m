clc;
initial_vel = 8:0.1:10;
tar_dist = 1.5:0.1:6.5;

chart = zeros(length(initial_vel), length(tar_dist));
n = length(initial_vel)* length(tar_dist);
s_vel = zeros(n,1);
s_dist = zeros(n, 1);
s_result = zeros(n, 1);
error = zeros(n, 1);

m = 1;
for i = 1:length(tar_dist)
    for j = 1:length(initial_vel)
        [value, is_valid ]= ballistic_find_angle(initial_vel(j), tar_dist(i));
        if(is_valid)
            chart(j,i) = value;
            s_vel(m) = initial_vel(j);
            s_dist(m) = tar_dist(i);
            s_result(m) = value;
        end
        m = m + 1;
    end
end

figure
f = fit([ s_vel, s_dist], s_result, 'poly44')
plot(f, [s_vel, s_dist], s_result)

for s = 1:(m-1)
    error(s) = s_result(s) - f(s_vel(s), s_dist(s));
end
figure
plot(f, [s_vel, s_dist], 60*(error/(pi/180/60)))

%plot(tar_dist, chart(9,:));