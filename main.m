clear
clc
close all

global previous_error_v_x previous_error_v_y previous_error_v_z
global sum_error_v_x sum_error_v_y sum_error_v_z

previous_error_v_x = 0;
previous_error_v_y = 0;
previous_error_v_z = 0;

sum_error_v_x = 0;
sum_error_v_y = 0;
sum_error_v_z = 0;


simTime = 20;
step = 0.01;

states = zeros(simTime/step,12);
X_r = zeros(simTime/step,3);
V_r = zeros(simTime/step,3);
 time = zeros(simTime/step,1);
 for i = 1:size(states,1)-1
     time(i) = step*i;
     if i == 1
         states(i,:) = [0 0 -1.5,0,0,0,0,0,0,0,0,0];
     end

     
     if time(i)<20
         X_r(i,:) = [1,0,-1.5];
     else
         X_r(i,:) = [3*sin(1/3*(time(i)-20))+20 3*cos(1/3*(time(i)-20))-3 -1.5];
     end

     [V_r(i,:)] = positionController(X_r(i,:),states(i,1:3));
%      
     V_r(i,:) = [0.5 0  0];
     %--------------------------
     % sensor model
     V_m = states(i,4:6);
     psi_r = 0;
     psi_m = states(i,11);
     %----------------------------
     [attitude_cmd,thrust_cmd] = velocityController(V_r(i,:),V_m,psi_m);
     states(i+1,:) = droneModel(states(i,:),[attitude_cmd psi_r thrust_cmd],step);
     %     states(i+1,:) = droneModel(states(i,:),[0/180*pi -5/180*pi 0  -9.8],step);
 end

figure(1)
subplot(3,1,1)
grid on;
 plot(time(1:end-1),X_r(1:end-1,1));
hold on
plot(time(1:end-1),states(1:end-1,1));
ylabel('x [m]');
subplot(3,1,2)
grid on;
 plot(time(1:end-1),X_r(1:end-1,2));
hold on
plot(time(1:end-1),states(1:end-1,2));
ylabel('y [m]');
subplot(3,1,3)
grid on;
 plot(time(1:end-1),X_r(1:end-1,3));
hold on
plot(time(1:end-1),states(1:end-1,3));
xlabel('time[s]');
ylabel('z [m]');

figure(3)
subplot(3,1,1)
grid on;
plot(time(1:end-1),states(1:end-1,7)/pi*180);
ylabel('phi [deg]');
subplot(3,1,2)
grid on;
plot(time(1:end-1),states(1:end-1,9)/pi*180);
ylabel('theta [deg]');
subplot(3,1,3)
grid on;
plot(time(1:end-1),states(1:end-1,11)/pi*180);
ylabel('psi [deg]');
xlabel('time[s]');

figure(4)
plot3(states(1:end-1,1),states(1:end-1,2),-states(1:end-1,3));
grid on
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis equal

figure(2)
subplot(3,1,1)
grid on;
plot(time(1:end-1),V_r(1:end-1,1));
hold on
plot(time(1:end-1),states(1:end-1,4));
ylabel('v_x [m/s]');
subplot(3,1,2)
grid on;
plot(time(1:end-1),V_r(1:end-1,2));
hold on
plot(time(1:end-1),states(1:end-1,5));
ylabel('v_y [m/s]');
subplot(3,1,3)
grid on;
plot(time(1:end-1),V_r(1:end-1,3));
hold on
plot(time(1:end-1),states(1:end-1,6));
xlabel('time[s]');
ylabel('v_z [m/s]');

temp = 1;