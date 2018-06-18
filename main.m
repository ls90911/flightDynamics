clear
clear global
clc
close all
global GT

generateGroundTruthData();
%verifyAngularVelocityAndSpecificForce(GT);
[SENSOR] = generateSensorReading(GT);

EKF_6_STATES(GT,SENSOR);

temp = 1;