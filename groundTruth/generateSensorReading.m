function [SENSOR] = generateSensorReading(GT)

SIGMA_GYRO = 0.5/180*pi;
SIGMA_ACC = 5;

BIAS_GYRO = 0;
BIAS_ACC = 0;

for i = 1:length(GT.TIME)
    SENSOR.P(i) = GT.P(i) + normrnd(0,SIGMA_GYRO)+BIAS_GYRO;
    SENSOR.Q(i) = GT.Q(i) + normrnd(0,SIGMA_GYRO)+BIAS_GYRO;
    SENSOR.R(i) = GT.R(i) + normrnd(0,SIGMA_GYRO)+BIAS_GYRO;
    SENSOR.AX(i) = GT.AX(i) + normrnd(0,SIGMA_ACC)+BIAS_ACC;
    SENSOR.AY(i) = GT.AY(i) + normrnd(0,SIGMA_ACC)+BIAS_ACC;
    SENSOR.AZ(i) = GT.AZ(i) + normrnd(0,SIGMA_ACC)+BIAS_ACC;
end

% figure(5)
% subplot(2,1,1)
% grid on;
% plot(GT.TIME,GT.Q/pi*180);
% hold on
% plot(GT.TIME,SENSOR.Q/pi*180,'*r');
% ylabel('Q [deg/s]');
% subplot(2,1,2)
% grid on;
% plot(GT.TIME,GT.AZ);
% hold on
% plot(GT.TIME,SENSOR.AZ);
% ylabel('AZ [m/s^2]');


end