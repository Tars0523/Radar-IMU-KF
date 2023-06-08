clear;
clc;
close all;

%% '/sbg/gps_pos' for True Value%%
bagselect = rosbag('SJU_Data_Local_Global.bag');
bSel = select(bagselect,'Topic','/sbg/gps_vel');
msgStructs = readMessages(bSel,'DataFormat','struct');

for i = 1:length(msgStructs)    
    vel_x = msgStructs{i}.Velocity.X;
    vel_y = msgStructs{i}.Velocity.Y;
    vel_z = msgStructs{i}.Velocity.Z;
    vel(i) = sqrt(vel_x^2 + vel_y^2 +vel_z^2);
end
%% imu from sbg 

bSel = select(bagselect,'Topic','/sbg/imu_data');
msgStructs = readMessages(bSel,'DataFormat','struct');
for i = 1:length(msgStructs)
    accel_X(i) = msgStructs{i}.Accel.X;
    accel_Y(i) = msgStructs{i}.Accel.Y;
    accel_Z(i) = msgStructs{i}.Accel.Z;
    gyro_X(i) = msgStructs{i}.Gyro.X;
    gyro_Y(i) = msgStructs{i}.Gyro.Y;
    gyro_Z(i) = msgStructs{i}.Gyro.Z;
end

%% odom(car wheel not RINS) global cloud point
bSel = select(bagselect,'Topic','/out_cloud');
msgStructs = readMessages(bSel,'DataFormat','struct');

for i = 1 : length(msgStructs)
    radial_speed(i,1:length(msgStructs{i}.Channels(1).Values)) = msgStructs{i}.Channels(1).Values;
    for j = 1 : length(msgStructs{i}.Points)
        if isempty(msgStructs{i}.Points)
            global_X(i,j) = 0;
            global_Y(i,j) = 0;
            global_Z(i,j) = 0;
            continue;
        end
        global_X(i,j) = msgStructs{i}.Points(j).X;
        global_Y(i,j) = msgStructs{i}.Points(j).Y;
        global_Z(i,j) = msgStructs{i}.Points(j).Z;
    end
end
%% local cloud point
bSel = select(bagselect,'Topic','/local_out_cloud');
msgStructs = readMessages(bSel,'DataFormat','struct');
for i = 1 : length(msgStructs)
    radial_speed(i,1:length(msgStructs{i}.Channels(1).Values)) = msgStructs{i}.Channels(1).Values;
    pcl_RCS(i,1:length(msgStructs{i}.Channels(1).Values)) = msgStructs{i}.Channels(2).Values;
    pcl_SNR(i,1:length(msgStructs{i}.Channels(1).Values)) = msgStructs{i}.Channels(3).Values;
    for j = 1 : length(msgStructs{i}.Points)
        if isempty(msgStructs{i}.Points)
            local_X(i,j) = 0;
            local_Y(i,j) = 0;
            local_Z(i,j) = 0;
            continue;
        end
        local_X(i,j) = double(msgStructs{i}.Points(j).X);
        local_Y(i,j) = double(msgStructs{i}.Points(j).Y);
        local_Z(i,j) = double(msgStructs{i}.Points(j).Z);
        pcl_length(i)=length(msgStructs{i}.Points);
    end
end

scatter3(local_X(1,:)',local_Y(1,:)',local_Z(1,:)','k');
xlabel("X");
ylabel("Y");
zlabel("Z");
hold on;
scatter3(local_X(2,:)',local_Y(2,:)',local_Z(2,:)','red');

bSel = select(bagselect,'Topic','/odom');
msgStructs = readMessages(bSel,'DataFormat','struct');
for i = 1 : length(msgStructs)
    odom_pose_x(i) = msgStructs{i}.Pose.Pose.Position.X;
    odom_pose_y(i) = msgStructs{i}.Pose.Pose.Position.Y;
    odom_vel_x(i) = msgStructs{i}.Twist.Twist.Linear.X;
    odom_vel_y(i) = msgStructs{i}.Twist.Twist.Linear.Y;
    odom_vel_z(i) = msgStructs{i}.Twist.Twist.Linear.Z;
end


bSel = select(bagselect,'Topic','/sbg/ekf_nav');
msgStructs = readMessages(bSel,'DataFormat','struct');
for i = 1 : length(msgStructs)
    EKF_v_x(i) = msgStructs{i}.Velocity.X;
    EKF_v_y(i) = msgStructs{i}.Velocity.Y;
    EKF_v_z(i) = msgStructs{i}.Velocity.Z;
end



