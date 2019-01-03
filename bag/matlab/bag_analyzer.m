bag = rosbag('../circle_diff.bag');


% %% acquire odom data
% odom = select(bag, 'Topic', '/elektron/mobile_base_controller/odom');
% ts_odom_X = timeseries(odom, 'Pose.Pose.Position.X');
% ts_odom_Y = timeseries(odom, 'Pose.Pose.Position.Y');
% ts_odom_theta = timeseries(odom, 'Twist.Twist.Angular.Z');
% 
% %% acquire laser data
% odom = select(bag, 'Topic', '/elektron/mobile_base_controller/odom');
% ts_laser_X = timeseries(odom, 'Pose.Pose.Position.X');
% ts_laser_Y = timeseries(odom, 'Pose.Pose.Position.Y');
% ts_laser_theta = timeseries(odom, 'Twist.Twist.Angular.Z');
% 
% %% acquire gazebo data
% gazebo = select(bag, 'Topic', '/gazebo_odom');
% ts_gazebo_X = timeseries(gazebo, 'Pose.Pose.Position.X');
% ts_gazebo_Y = timeseries(gazebo, 'Pose.Pose.Position.Y');
% ts_gazebo_theta = timeseries(gazebo, 'Twist.Twist.Angular.Z');

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


%% plot errors
figure
hold on;
grid on;
grid minor;
plot(ts_oerr_theta);
title('Diff drive controller error - angle theta');
legend('diff_{}drive_{}controller');
hold off;

figure
hold on;
grid on;
grid minor;
plot(ts_lerr_theta)
title('Laser scaner error - angle theta');
legend('laser');
hold off;
% 
% figure
% hold on;
% grid on;
% grid minor;
% plot(ts_oerr_X);
% title('Diff drive controller error - X axis');
% legend('diff_{}drive_{}controller');
% hold off;
% 
% figure
% hold on;
% grid on;
% grid minor;
% plot(ts_lerr_X)
% title('Laser scaner error - X axis');
% legend('laser');
% hold off;

% %% plot X data
% figure
% plot(ts_odom_X); 
% hold on; 
% grid on; 
% grid minor; 
% plot(ts_gazebo_X);
% title('Porównanie pozycji w współrzędnej X');
% legend('Robot', 'Reality');
% hold off;
% 
% %% plot Y data
% figure
% plot(ts_odom_Y); 
% hold on; 
% grid on; 
% grid minor; 
% plot(ts_gazebo_Y);
% title('Porównanie pozycji w współrzędnej Y');
% legend('Robot', 'Reality');
% hold off;
% 
% %% plot theta data
% figure
% plot(ts_odom_theta); 
% hold on; 
% grid on; 
% grid minor; 
% plot(ts_gazebo_theta);
% title('Porównanie pozycji w współrzędnej theta');
% legend('Robot', 'Reality');
% hold off;