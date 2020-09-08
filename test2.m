std_x=[];
std_y=[];
std_z=[];

x=[];
y=[];
z=[];

subplot(3,1,1)
% plot(local_err(1:200,1));
hold on
for i = 1:4
    plot(local_err(1:200,1+3*(i-1)));
    x(1:200,i) = local_err(1:200,1+3*(i-1));
end
% plot(local_err(1:200,4));
% plot(local_err(1:200,7));
% plot(local_err(1:200,10));



legend('Landmark 1','Landmark 2','Landmark 3','Landmark 4')
title('Trajectory of the global error state on x axis')
xlabel('Timestep')
ylabel('Error state')



subplot(3,1,2)
% plot(local_err(1:200,2));
hold on
% plot(local_err(1:200,5));
% plot(local_err(1:200,8));
% plot(local_err(1:200,11));

for i = 1:4
    plot(local_err(1:200,2+3*(i-1)));
    y(1:200,i) = local_err(1:200,2+3*(i-1));
end

title('Trajectory of the global error state on y axis')
xlabel('Timestep')
ylabel('Error state')



subplot(3,1,3)
% plot(local_err(1:200,3));
hold on
for i = 1:4
    plot(local_err(1:200,3+3*(i-1)));
    z(1:200,i) = local_err(1:200,3+3*(i-1));
end

% plot(local_err(1:200,6));
% plot(local_err(1:200,9));
% plot(local_err(1:200,12));
title('Trajectory of the global error state on z axis')
xlabel('Timestep')
ylabel('Error state')


for i = 1:200
    
    std_x(i) = std(x(i,:));
    std_y(i) = std(y(i,:));
    std_z(i) = std(z(i,:));
end
