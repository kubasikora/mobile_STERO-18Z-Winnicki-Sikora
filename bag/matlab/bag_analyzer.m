bag = rosbag('../no_odom.bag');


%% acquire odom data
odom = select(bag, 'Topic', '/elektron/mobile_base_controller/odom');
ts_odom_X = timeseries(odom, 'Pose.Pose.Position.X');
ts_odom_Y = timeseries(odom, 'Pose.Pose.Position.Y');
ts_odom_theta = timeseries(odom, 'Twist.Twist.Angular.Z');


%% acquire gazebo data
gazebo = select(bag, 'Topic', '/gazebo_odom');
ts_gazebo_X = timeseries(gazebo, 'Pose.Pose.Position.X');
ts_gazebo_Y = timeseries(gazebo, 'Pose.Pose.Position.Y');
ts_gazebo_theta = timeseries(gazebo, 'Twist.Twist.Angular.Z');


%% plot X data
figure
plot(ts_odom_X); 
hold on; 
grid on; 
grid minor; 
plot(ts_gazebo_X);
title('Porównanie pozycji w współrzędnej X');
legend('Robot', 'Reality');
hold off;


%% plot Y data
figure
plot(ts_odom_Y); 
hold on; 
grid on; 
grid minor; 
plot(ts_gazebo_Y);
title('Porównanie pozycji w współrzędnej Y');
legend('Robot', 'Reality');
hold off;

%% plot theta data
figure
plot(ts_odom_theta); 
hold on; 
grid on; 
grid minor; 
plot(ts_gazebo_theta);
title('Porównanie pozycji w współrzędnej theta');
legend('Robot', 'Reality');
hold off;