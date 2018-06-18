function [] = verifyAngularVelocityAndSpecificForce(GT)
global step
attGyro = zeros(length(GT.TIME),3);
attAcc = zeros(length(GT.TIME),3);

for i = 1:length(GT.TIME)-1
    phi = GT.PHI(i);
    theta = GT.THETA(i);
    psi = GT.PSI(i);
    dTheta = [1 0 -sin(theta);...
        0 cos(phi) cos(theta)*sin(phi);...
        0 -sin(phi) cos(theta)*cos(phi)]*[GT.P(i) GT.Q(i) GT.R(i)]';
    attGyro(i+1,:) = attGyro(i,:) + dTheta'*step;
    attAcc(i,:) = [atan2(-GT.AY(i),-GT.AZ(i)) atan2(GT.AX(i),sqrt(GT.AY(i)^2+GT.AZ(i)^2)) 0];
end

figure(4)
subplot(3,1,1)
grid on;
plot(GT.TIME,GT.PHI/pi*180);
hold on
plot(GT.TIME,attGyro(:,1)/pi*180);
plot(GT.TIME,attAcc(:,1)/pi*180);
ylabel('phi [deg]');
subplot(3,1,2)
grid on;
plot(GT.TIME,GT.THETA/pi*180);
hold on
plot(GT.TIME,attGyro(:,2)/pi*180);
plot(GT.TIME,attAcc(:,2)/pi*180);
ylabel('theta [deg]');
legend('GT','gyro','acc')
subplot(3,1,3)
grid on;
plot(GT.TIME,GT.PSI/pi*180);
hold on
plot(GT.TIME,attGyro(:,3)/pi*180);
plot(GT.TIME,attAcc(:,3)/pi*180);
ylabel('psi [deg]');
xlabel('time[s]');

end