%% ICP only rotation maybe accurate...?
%% ICP minumum 3 points,,,maybe only 3points near prediction of rotation maybe can make accurate ICP?
%% EKF...
%% maybe indoor accurate..? let's do with indoor radar...
%% maybe radar ransac ?? 



%% Radar Odom
pose = [0;0;0]; % init pose
 % gap between radar pcl data
predict_speed(1) = 0;
delta_t =  0.0549 ; % sbg imu cycle
v_x(1) = 0;
v_y(1) = 0;
%%[radial_speed] = Radial_relative(radial_speed,local_X,local_Y,local_Z,pcl_length);
%% accel and radar_pcl synchronized
[accel_X,accel_Y,accel_Z] = Synchron(accel_X,accel_Y,accel_Z,length(accel_X),length(local_X));
[Gyro_X,Gyro_Y,Gyro_Z] = Synchron(gyro_X,gyro_Y,gyro_Z,length(gyro_X),length(local_X));
accel_Z = accel_Z - 9.81; % gravity compensate
before_state = [0 0 0 0]';
optimal_state = [0 0 0 0]';
P = [5 0 0 0 ;...
     0 5 0 0 ;...
     0 0 5 0 ;...
     0 0 0 5 ];
Q = [3 0 0 0 ;...
     0 3 0 0 ;...
     0 0 1 0 ;...
     0 0 0 1 ];
R = [0.5];


%% Radial velocity to relative speed
[radial_speed] = Radial_relative(radial_speed,local_X,local_Y,local_Z,pcl_length);
gap  = 1 ;
for i = 1:gap:6439
    
    if i+gap > 6439
        break;
    end
    %% PCL length

    l = pcl_length(i); %before
    l__ = pcl_length(i+gap); %current
    
    %% before state
    before_state(1) = optimal_state(1);
    before_state(2) = optimal_state(2);
    before_state(3) = accel_X(i);
    before_state(4) = accel_Y(i);
    
    %% state transition
    A = [1 0 delta_t 0 ;...
         0 1 0 delta_t ;...
         0 0 1 0 ;...
         0 0 0 1 ];
    pred_state = A*before_state;
    
    %% observation model
    H = [pred_state(1)/sqrt(pred_state(1)^2+pred_state(2)^2) pred_state(2)/sqrt(pred_state(1)^2+pred_state(2)^2) 0 0];

    %% covariance pred
    P_ = A*P*A' + Q;
    
    %% compute kalman gain
    K = P_*H'*inv(H*P_*H'+R);
    
    %% Model transition (i+1 :current, i : before)
    measuerment_speed(i+1) = speed_prediction(optimal_state(1),optimal_state(2),delta_t,before_state(3),before_state(4),radial_speed(i+1,1:pcl_length(i+1)));
    z = measuerment_speed(i+1);
    %% update state
    optimal_state = pred_state + K*(z-H*pred_state);
    %% update covariance
    P = P_ - K*H*P_;
    %% Stack
    optimal_state_stack(i,:) = optimal_state;
    covariance_stack(i,1:4,1:4) = P;
end  

figure(1)
sync_pred_vel_gps_vel = interp1(linspace(1,length(vel),length(vel)),vel,linspace(1,length(vel),length(measuerment_speed)));
plot(sync_pred_vel_gps_vel,'red');
hold on;
plot(sqrt(optimal_state_stack(:,1).^2+optimal_state_stack(:,2).^2),'b');
legend("gps","radar imu");
title("radar imu vs gps");

%% ICP only rotation maybe accurate...?
%% ICP minumum 3 points,,,maybe only 3points near prediction of rotation maybe can make accurate ICP?
%% EKF...
















