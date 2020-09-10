std_x=[];
std_y=[];
std_z=[];

x=[];
y=[];
z=[];


subplot(3,1,1)
% plot(local_err(1:200,1));
hold on
for i = 12:15
    plot(abs(local_err(2:200,1+3*(i-1))));
    x(1:200,i) = local_err(1:200,1+3*(i-1));
end
% plot(local_err(1:200,4));
% plot(local_err(1:200,7));
% plot(local_err(1:200,10));



% legend('Landmark 1','Landmark 2','Landmark 3','Landmark 4')
% title('Trajectory of the global error state on x axis')
xlabel('Time(s)')
ylabel('Error on x axis')
xticks([1 20 40 60 80 100])
xticklabels({'0','2','4','6','8','10'})
 ylim([0,1])
xlim([1,100])
% axis([0 10 0 2])


subplot(3,1,2)
% plot(local_err(1:200,2));
hold on
% plot(local_err(1:200,5));
% plot(local_err(1:200,8));
% plot(local_err(1:200,11));

for i = 12:15
    plot(abs(local_err(2:200,2+3*(i-1))));
    y(1:200,i) = local_err(1:200,2+3*(i-1));
end

% title('Trajectory of the global error state on y axis')
xlabel('Time(s)')
ylabel('Error on y axis')
ylim([0,1])
xlim([1,100])
% axis([0 10 0 2])
xticks([1 20 40 60 80 100])
xticklabels({'0','2','4','6','8','10'})


subplot(3,1,3)
% plot(local_err(1:200,3));
hold on
for i = 12:15
    plot(abs(local_err(2:200,3+3*(i-1))));
    z(1:200,i) = local_err(1:200,3+3*(i-1));
end

% plot(local_err(1:200,6));
% plot(local_err(1:200,9));
% plot(local_err(1:200,12));
% title('Trajectory of the global error state on z axis')
xlabel('Time(s)')
ylabel('Error on z axis')
% ylim([0,2])
xlim([1,100])
% axis([0 10 0 2])
xticks([1 20 40 60 80 100])
xticklabels({'0','2','4','6','8','10'})


%%
figure
plot(log(lyapn));
xlabel('Time(s)')
ylabel('Log(L)')
% ylim([0,2])
xlim([1,200])
% axis([0 10 0 2])
xticks([1 40 80 120 160 200])
xticklabels({'0','2','4','6','8','10'})


%%

figure
plot(s_1,'r','LineWidth',2);
hold on
plot(s_2,'b','LineWidth',2);

xlabel('Time(s)')
ylabel("Error State")

xlim([1,200])
% axis([0 10 0 2])
xticks([1 40 80 120 160 200])
xticklabels({'0','2','4','6','8','10'})