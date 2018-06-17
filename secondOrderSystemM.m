function [] = secondOrderSystemM()
%SECONDORDERSYSTEM Summary of this function goes here
%   Detailed explanation goes here
simTime = 10;
simStep = 0.01;
omega = 10;
xi = 0.9;

states = zeros(simTime/simStep,2);
time = zeros(simTime/simStep,1);

for i = 1:size(states,1)-1
    time(i+1) = (i+1)*simStep;
    dx = [0 1;-omega^2 -2*xi*omega]*states(i,:)'+[0 omega^2]';
    states(i+1,:) = states(i,:)+dx'*simStep;
end

plot(time,states(:,1));
grid on

end

