clear variables
close all

% Creating Serial Object
port_list = serialportlist("available");
s = serial("/dev/tty.usbmodem918FF88B2","BaudRate",115200);
% for windows pc
% s = serial("COM#","BaudRate",115200);

% Opening Serial Port
fopen(s);

window_size = 149;

dt = 0.008;

A_x = [1 dt;0 1]; A_y = [1 dt;0 1]; A_z = [1 dt;0 1];
H_x = [0 1]; H_y = [0 1]; H_z = [0 1];

v_x(1) = 0; v_y(1) = 0; v_z(1) = 0;
a_x(1) = 0; a_y(1) = 0; a_z(1) = 0;

i = 1;

I = eye(2);

xpri_x(:,1) = [v_x(1);a_x(1)]; xpri_y(:,1) = [v_y(1);a_y(1)]; xpri_z(:,1) = [v_z(1);a_z(1)];
x_x(:,1) = [v_x(1); a_x(1)]; x_y(:,1) = [v_y(1); a_y(1)]; x_z(:,1) = [v_z(1); a_z(1)];

Q_x = 0.2*[1 0;0 1]; Q_y = 0.2*[1 0;0 1]; Q_z = 0.3*[1 0;0 1];
% R_x = 0.0002*[1 0;0 1]; R_y = 0.0002*[1 0;0 1]; R_z = 0.0003*[1 0;0 1];
R_x = 0.2; R_y = 0.2; R_z = 0.3;

Ppri_x(:,:,1) = 10*eye(2); Ppri_y(:,:,1) = 10*eye(2); Ppri_z(:,:,1) = 10*eye(2);
P_x(:,:,1) = 10*eye(2); P_y(:,:,1) = 10*eye(2); P_z(:,:,1) = 10*eye(2);

xvel(i) = 0; yvel(i) = 0; zvel(i) = 0;

while(1)
% pause(0.01);
try
    imu_data(i,:) = str2num(fscanf(s));
catch
    
end
imu_data(i,:) = str2num(fscanf(s));
xacc(i+1) = imu_data(i,1);
xvel(i+1) = xvel(i) + dt*xacc(i+1);
yacc(i+1) = imu_data(i,2);
yvel(i+1) = yvel(i) + dt*yacc(i+1);
zacc(i+1) = imu_data(i,3);
zvel(i+1) = zvel(i) + dt*zacc(i+1);
%% x kalman filter
x_z_x(:,i) = [xvel(i+1); xacc(i+1)];
Z_x(:,i) = H_x*x_z_x(:,i);
xpri_x(:,i+1) = A_x*x_x(:,i);
Ppri_x(:,:,i+1) = A_x*P_x(:,:,i)*transpose(A_x) + Q_x;
K_x(:,i) = Ppri_x(:,:,i+1)*transpose(H_x)*(inv(H_x*Ppri_x(:,:,i+1)*transpose(H_x) + R_x));
P_x(:,:,i+1) = (I-K_x(:,i)*H_x)*Ppri_x(:,:,i+1);
x_x(:,i+1) = xpri_x(:,i+1)+K_x(:,i)*(Z_x(:,i) - H_x*xpri_x(:,i+1));

%% x kalman filter
x_z_y(:,i) = [yvel(i+1); yacc(i+1)];
Z_y(:,i) = H_y*x_z_y(:,i);
xpri_y(:,i+1) = A_y*x_y(:,i);
Ppri_y(:,:,i+1) = A_y*P_y(:,:,i)*transpose(A_y) + Q_y;
K_y(:,i) = Ppri_y(:,:,i+1)*transpose(H_y)*(inv(H_y*Ppri_y(:,:,i+1)*transpose(H_y) + R_y));
P_y(:,:,i+1) = (I-K_y(:,i)*H_y)*Ppri_y(:,:,i+1);
x_y(:,i+1) = xpri_y(:,i+1)+K_y(:,i)*(Z_y(:,i) - H_y*xpri_y(:,i+1));

%% x kalman filter
x_z_z(:,i) = [zvel(i+1); zacc(i+1)];
Z_z(:,i) = H_z*x_z_z(:,i);
xpri_z(:,i+1) = A_z*x_z(:,i);
Ppri_z(:,:,i+1) = A_z*P_z(:,:,i)*transpose(A_z) + Q_z;
K_z(:,i) = Ppri_z(:,:,i+1)*transpose(H_z)*(inv(H_z*Ppri_z(:,:,i+1)*transpose(H_z) + R_z));
P_z(:,:,i+1) = (I-K_z(:,i)*H_z)*Ppri_z(:,:,i+1);
x_z(:,i+1) = xpri_z(:,i+1)+K_z(:,i)*(Z_z(:,i) - H_z*xpri_z(:,i+1));

%plots
figure(1);
set(gcf,'units','normalized','outerposition',[0 0 1 1])
%     if i<=window_size+1
        subplot(311)
        plot(x_z_x(1,:),'b');
        hold on
        plot(x_x(1,:),'r'),title("X Velocity Estimated");
        subplot(312)
        plot(x_z_y(1,:),'b'),title("Y Velocity IMU");
        hold on
        plot(x_y(1,:),'r'),title("Y Velocity Estimated");
        subplot(313)
        plot(x_z_z(1,:),'b'),title("Z Velocity IMU");
        hold on
        plot(x_z(1,:),'r'),title("Z Velocity Estimated");
%     else
%         subplot(311)
%         plot(x_z_x(1,end-window_size:end),'b'),title("X Velocity IMU");
%         hold on
%         plot(x_x(1,end-window_size:end),'r'),title("X Velocity Estimated");
%         subplot(312)
%         plot(x_z_y(1,end-window_size:end),'b'),title("Y Velocity IMU");
%         hold on
%         plot(x_y(1,end-window_size:end),'r'),title("Y Velocity Estimated");
%         subplot(313)
%         plot(x_z_z(1,end-window_size:end),'b'),title("Z Velocity IMU");
%         hold on
%         plot(x_z(1,end-window_size:end),'r'),title("Z Velocity Estimated");
%     end
    
    i=i+1;
end