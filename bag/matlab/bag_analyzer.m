bag = rosbag('../circle_tune.bag');


%% acquire odom data
odom = select(bag, 'Topic', '/elektron/mobile_base_controller/odom');
ts_odom_X = timeseries(odom, 'Pose.Pose.Position.X');
ts_odom_Y = timeseries(odom, 'Pose.Pose.Position.Y');
ts_odom_theta = timeseries(odom, 'Twist.Twist.Angular.Z');
 
%% acquire laser data
laser = select(bag, 'Topic', '/pose2D');
ts_laser_X = timeseries(laser, 'X');
ts_laser_Y = timeseries(laser, 'Y');
ts_laser_theta = timeseries(laser, 'Theta');
 
%% acquire gazebo data
gazebo = select(bag, 'Topic', '/gazebo_odom');
ts_gazebo_X = timeseries(gazebo, 'Pose.Pose.Position.X');
ts_gazebo_Y = timeseries(gazebo, 'Pose.Pose.Position.Y');
ts_gazebo_theta = timeseries(gazebo, 'Twist.Twist.Angular.Z');

%% acquire odom error
ts_oerr = select(bag, 'Topic', '/stero/errors/odom');
ts_oerr_X = timeseries(ts_oerr, 'X');
ts_oerr_Y = timeseries(ts_oerr, 'Y');
ts_oerr_theta = timeseries(ts_oerr, 'Theta');
 
%% acquire laser error
ts_lerr = select(bag, 'Topic', '/stero/errors/laser');
ts_lerr_X = timeseries(ts_lerr, 'X');
ts_lerr_Y = timeseries(ts_lerr, 'Y');
ts_lerr_theta = timeseries(ts_lerr, 'Theta');


%% plot errors theta
figure
hold on;
grid on;
grid minor;
plot(ts_oerr_theta);
title('Tune controller error - angle theta');
legend('Tune controller error');
hold off;

name='circle_tune_theta';
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
print(fig,name,'-dpdf');
 

figure
hold on;
grid on;
grid minor;
plot(ts_lerr_theta)
title('Laser scaner error - angle theta');
legend('laser');
hold off;

name='circle_diff_laser_theta';
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
print(fig,name,'-dpdf');
%% plot errors x

figure
hold on;
grid on;
grid minor;
plot(ts_oerr_X);
title('Tune controller error - X axis');
legend('Tune controller error');
hold off;

name='circle_tune_x';
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
print(fig,name,'-dpdf');

figure
hold on;
grid on;
grid minor;
plot(ts_lerr_X)
title('Laser scaner error - X axis');
legend('laser');
hold off;

name='circle_tune_laser_x';
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
print(fig,name,'-dpdf');

%% plot errors y
figure
hold on;
grid on;
grid minor;
plot(ts_lerr_Y)
title('Laser scaner error - Y axis');
legend('laser');
hold off;

name='circle_tune_laser_y';
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
print(fig,name,'-dpdf');

figure
hold on;
grid on;
grid minor;
plot(ts_oerr_Y);
title('Tune controllerr error - Y axis');
legend('Tune controller error');
hold off;

name='circle_tune_y';
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
print(fig,name,'-dpdf');

% %% plot X data
% f = figure;
% hold on; 
% grid on; 
% grid minor; 
% plot(ts_odom_X); 
% plot(ts_gazebo_X);
% %plot(ts_laser_X);
% title('Porównanie pozycji w współrzędnej X');
% legend('odom', 'gazebo')'%, 'laser');
% hold off;
% 
% name='lab1xccw';
% fig = gcf;
% fig.PaperPositionMode = 'auto';
% fig_pos = fig.PaperPosition;
% fig.PaperSize = [fig_pos(3) fig_pos(4)];
% print(fig,name,'-dpdf');
%  
% %% plot Y data
% f = figure; 
% hold on; 
% grid on; 
% grid minor; 
% plot(ts_odom_Y); 
% plot(ts_gazebo_Y);
% %plot(ts_laser_Y);
% title('Porównanie pozycji w współrzędnej Y');
% legend('odom', 'gazebo');%, 'laser');
% hold off;
% 
% name='lab1yccw';
% fig = gcf;
% fig.PaperPositionMode = 'auto';
% fig_pos = fig.PaperPosition;
% fig.PaperSize = [fig_pos(3) fig_pos(4)];
% print(fig,name,'-dpdf');
%  


% %% plot theta data
% figure;
% hold on; 
% grid on; 
% grid minor; 
% plot(ts_odom_theta); 
% plot(ts_gazebo_theta);
% %plot(ts_laser_theta);
% title('Porównanie pozycji w współrzędnej kątowej');
% legend('odom', 'gazebo');%, 'laser');
% 
% name='lab1thc   cw';
% fig = gcf;
% fig.PaperPositionMode = 'auto';
% fig_pos = fig.PaperPosition;
% fig.PaperSize = [fig_pos(3) fig_pos(4)];
% print(fig,name,'-dpdf');
% 
% hold off;