function [] = generateGroundTruthData()
global i step GT 

FIGURE = 0;

global previous_error_v_x previous_error_v_y previous_error_v_z
global sum_error_v_x sum_error_v_y sum_error_v_z

previous_error_v_x = 0;
previous_error_v_y = 0;
previous_error_v_z = 0;

sum_error_v_x = 0;
sum_error_v_y = 0;
sum_error_v_z = 0;


simTime = 30;
step = 0.02;
GT.STEP = step;
states = zeros(simTime/step,12);
X_r = zeros(simTime/step,3);
V_r = zeros(simTime/step,3);
 time = zeros(simTime/step,1);
 
 
 for i = 1:size(states,1)-1
     time(i) = step*i;
     if i == 1
         states(i,:) = [0 0 -1.5,0,0,0,0,0,0,0,0,0];
     end
     
     
%      if time(i) < 5
%          V_r(i,:) = [0.5 -0.5 0];
%      elseif time(i) < 10
%          V_r(i,:) = [-0.5 0.5 0];
%      elseif time(i) < 15
%          V_r(i,:) = [-1 -0.5 0];
%      elseif time(i) < 20
%      else 
%          V_r(i,:) = [0.5 0  0];
%      end
if time(i) < 10
    X_r(i,:) = [5*sin(pi/4*time(i)) -5*sin(pi/3*time(i)) -1.5];
    X_m = states(i,1:3);
    [V_r(i,:)] = positionController(X_r(i,:),X_m);
elseif time(i) < 15
    V_r(i,:) = [-0.5 0.5 0];
elseif time(i) < 20
    V_r(i,:) = [-1 -0.5 0];
elseif time(i) < 25
else
    V_r(i,:) = [0.5 0  0];
end

     %--------------------------
     % sensor model
     V_m = states(i,4:6);
     psi_r = 0;
     psi_m = states(i,11);
     %----------------------------
     [attitude_cmd,thrust_cmd] = velocityController(V_r(i,:),V_m,psi_m);
     %attitude_cmd = [30/180*pi*sin(pi/4*time(i)) 20/180*pi*sin(pi/3*time(i))]
     states(i+1,:) = droneModel(states(i,:),[attitude_cmd psi_r thrust_cmd],step);
 end

 if FIGURE == 1
figure(1)
subplot(3,1,1)
grid on;
 plot(GT.TIME,X_r);
hold on
plot(GT.TIME,GT.X);
ylabel('x [m]');
subplot(3,1,2)
grid on;
 plot(GT.TIME,X_r);
hold on
plot(GT.TIME,GT.Y);
ylabel('y [m]');
subplot(3,1,3)
grid on;
 plot(GT.TIME,X_r);
hold on
plot(GT.TIME,GT.Z);
xlabel('time[s]');
ylabel('z [m]');

figure(3)
subplot(3,1,1)
grid on;
plot(GT.TIME,GT.PHI/pi*180);
ylabel('phi [deg]');
subplot(3,1,2)
grid on;
plot(GT.TIME,GT.THETA/pi*180);
ylabel('theta [deg]');
subplot(3,1,3)
grid on;
plot(GT.TIME,GT.PSI/pi*180);
ylabel('psi [deg]');
xlabel('time[s]');

figure(4)
plot3(GT.X,GT.Y,-GT.Z);
grid on
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis equal

figure(2)
subplot(3,1,1)
grid on;
plot(GT.TIME,V_r);
hold on
plot(GT.TIME,GT.VX);
ylabel('v_x [m/s]');
subplot(3,1,2)
grid on;
plot(GT.TIME,V_r);
hold on
plot(GT.TIME,GT.VY);
ylabel('v_y [m/s]');
subplot(3,1,3)
grid on;
plot(GT.TIME,V_r);
hold on
plot(GT.TIME,GT.VZ);
xlabel('time[s]');
ylabel('v_z [m/s]');
 end

end