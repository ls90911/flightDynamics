function [newStates] = droneModel(currentStates,inputs,timeStep)
dragMatrix = [-0.5 0 0;0 -0.5 0;0 0 -0.5];
omega = 10;
xi = 0.9;
g = 9.8;

v = currentStates(4:6)';
phi = currentStates(7);
phiVector = currentStates(7:8)';
theta = currentStates(9);
thetaVector = currentStates(9:10)';
psi = currentStates(11);
psiVector = currentStates(11:12)';

phi_cmd = inputs(1);
theta_cmd = inputs(2);
psi_cmd = inputs(3);
T = inputs(4);

R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
R_B_E = R_E_B';

dPosition = v;
dVelocity = [0 0 g]' +R_B_E*[0 0 T]'+R_B_E*dragMatrix*R_E_B*v;

A = [0 1;-omega^2 -2*xi*omega];
B = [0;omega^2];

dPhiVector = A*phiVector+B*phi_cmd;
dThetaVector = A*thetaVector+B*theta_cmd;
dPsiVector = A*psiVector+B*psi_cmd;

dx = [dPosition;dVelocity;dPhiVector;dThetaVector;dPsiVector];
newStates = currentStates+dx'*timeStep;
end