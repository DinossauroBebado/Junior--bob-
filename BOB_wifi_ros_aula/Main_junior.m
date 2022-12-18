% para poder utilizar este scrip, primeiramente inicie o turtlesim node com
% o comando: 
% rosrun turtlesim turtlesim_node

% rosinit; 

% ------- config publisher
msg_twist = rosmessage('geometry_msgs/Twist') ;
pub_twist = rospublisher("/cmd_vel",'geometry_msgs/Twist');

% ------- config subscriber
sub_odom = rossubscriber("odom");
odom_data = receive(sub_odom,10); 

% setpoint xy
target_x = 3.5;
target_y = 3.5;

% constantes PID linaer 
kp_linear = 0.5;
ki_linear = 0.1; 
kd_linear = 0.1; 

% constantes PID angular
kp_angular = 0.1;
ki_angular = 0.1; 
kd_angulat = 0.1; 

% variavais para calculo deltaT
elapsedTime = 0; 
previous_time = clock;

previous_error_linear = 0;
previous_error_angular = 0; 

vel_linear_integral = 0;
vel_angular_integral = 0; 

error_linear =  hypot((target_x - odom_data.Pose.Pose.Position.X),(target_y - odom_data.Pose.Pose.Position.Y)); 
error_angular =  99;   % fazer com que o scrip entre na condição while abaixo

while(abs(error_angular) > pi/10 && abs(error_linear) > 1)
    
    odom_data = receive(sub_odom,10); 
        
    setpoint_angle = atan2((target_x-odom_data.Pose.Pose.Position.X),(target_y-odom_data.Pose.Pose.Position.Y));

    error_linear =  hypot((target_y - odom_data.Pose.Pose.Position.Y),(target_x - odom_data.Pose.Pose.Position.X));
        
    error_angular = setpoint_angle - odom_data.Pose.Pose.Orientation.Z;

    % condição para complementar o angulo caso necessário
    if(error_angular<0 && odom_data.Pose.Pose.Orientation.Z > 0)
        error_angular = (setpoint_angle - odom_data.Pose.Pose.Orientation.Z) + 2*pi; 
    end

    % Delta T para calculo de derivada e integral
    elapsedTime = etime(clock, previous_time); 
    
    % ------- integral 
    vel_linear_integral = (error_linear*ki_linear*elapsedTime) + vel_linear_integral;
    vel_angular_integral = (error_angular*ki_angular*elapsedTime) + vel_angular_integral; 

    % ------- devidada 
    vel_linear_derivative = (error_linear- previous_error_linear)/elapsedTime; 
    vel_angular_derivative = (error_angular - previous_error_angular)/elapsedTime; 

    % ------- proporcional 
    vel_linear_proporcional = error_linear*kp_linear;
    vel_angular_proporcional = error_angular*kp_angular;

    % ------ calculo PID 
    vel_linear = vel_linear_proporcional + vel_linear_integral + vel_linear_derivative; 
    vel_angular = vel_angular_proporcional + vel_angular_integral + vel_angular_derivative; 
    
    disp(error_linear);

    msg_twist.Linear.X = vel_linear;
    msg_twist.Angular.Z = 0;

    send(pub_twist,msg_twist);
    
    previous_time = clock; 

    previous_error_angular = error_angular; 
    previous_error_linear = error_linear;

end

msg_twist.Linear.X =0;
msg_twist.Angular.Z = 0;
    
send(pub_twist,msg_twist);





