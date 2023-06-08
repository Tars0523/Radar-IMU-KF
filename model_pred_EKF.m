%% ICP only rotation maybe accurate...?
%% ICP minumum 3 points,,,maybe only 3points near prediction of rotation maybe can make accurate ICP?
%% EKF...
%% maybe indoor accurate..? let's do with indoor radar...
%% maybe radar ransac ?? 



%% Radar Odom
pose = [0;0;0]; % init pose
 % gap between radar pcl data
predict_speed(1) = 0;
delta_t =  0.0550 ; % sbg imu cycle
v_x(1) = 0;
v_y(1) = 0;
%%[radial_speed] = Radial_relative(radial_speed,local_X,local_Y,local_Z,pcl_length);
%% accel and radar_pcl synchronized
[accel_X,accel_Y,accel_Z] = Synchron(accel_X,accel_Y,accel_Z,length(accel_X),length(local_X));
[Gyro_X,Gyro_Y,Gyro_Z] = Synchron(gyro_X,gyro_Y,gyro_Z,length(gyro_X),length(local_X));
accel_Z = accel_Z - 9.81; % gravity compensate
before_state = [0 0 0 0 0]';
optimal_state = [0 0 0 0 0]';
P = [5 0 0 0 0;...
     0 5 0 0 0;...
     0 0 5 0 0;...
     0 0 0 5 0;...
     0 0 0 0 5];
Q = [1 0;...
     0 1];
R = [1];


%% Radial velocity to relative speed
%[radial_speed] = Radial_relative(radial_speed,local_X,local_Y,local_Z,pcl_length);
gap  = 1 ;
for i = 1:gap:6439
    
    if i+gap > 6439
        break;
    end
    %% PCL length

    l = pcl_length(i); %before
    l__ = pcl_length(i+gap); %current
    
    
    %% state pred
    pred_state(1) = sqrt((optimal_state(2)+delta_t*accel_X(i))^2+(optimal_state(3)+delta_t*accel_Y(i))^2);
    pred_state(2) = optimal_state(2)+delta_t*accel_X(i);
    pred_state(3) = optimal_state(3)+delta_t*accel_Y(i);
    pred_state(4) = accel_X(i);
    pred_state(5) = accel_Y(i);
    
    %% model transition matrix
    F = [0 pred_state(2)/pred_state(1) pred_state(3)/pred_state(1) pred_state(2)/pred_state(1)*delta_t pred_state(3)/pred_state(1)*delta_t;...
        0 1 0 delta_t 0;...
        0 0 1 0 delta_t;...
        0 0 0 1 0;...
        0 0 0 0 1];
    
    %% model covariance matrix
    G = [0 0;...
         0 0;...
         0 0;...
         delta_t 0;...
         0 delta_t];


    %% observation model
    H = [1 pred_state(1)/sqrt(pred_state(1)^2+pred_state(2)^2) pred_state(2)/sqrt(pred_state(1)^2+pred_state(2)^2) 0 0];

    %% covariance pred
    P_ = F*P*F' + G*Q*G';
    
    %% compute kalman gain
    K = P_*H'*inv(H*P_*H'+R);
    
    %% Model transition (i+1 :current, i : before)
    measuerment_speed(i+1) = speed_prediction(optimal_state(2),optimal_state(3),delta_t,accel_X(i),accel_Y(i),radial_speed(i+1,1:pcl_length(i+1)));
    z = measuerment_speed(i+1);
    %% update state
    optimal_state = pred_state + K*(z-H*pred_state);
    %% update covariance
    P = P_ - K*H*P_;
    %% Stack
    optimal_state_stack(i,1:5) = optimal_state;
    covariance_stack(i,1:5,1:5) = P;
end  

figure(1)
sync_pred_vel_gps_vel = interp1(linspace(1,length(vel),length(vel)),vel,linspace(1,length(vel),length(measuerment_speed)));
plot(sync_pred_vel_gps_vel,'red');
hold on;
odom_Vel = sqrt(odom_vel_x.^2 + odom_vel_y.^2 + odom_vel_z.^2);
odom_sync = interp1(linspace(1,length(odom_Vel),length(odom_Vel)), odom_Vel, linspace(1,length(odom_Vel),length(measuerment_speed)));
plot(odom_sync,'k');
plot(optimal_state_stack(:,1),'b');
legend("gps imu","wheel odom","radar imu");
title("radar_imu vs odom vs gps_imu");

%% ICP only rotation maybe accurate...?
%% ICP minumum 3 points,,,maybe only 3points near prediction of rotation maybe can make accurate ICP?
%% EKF...
















