function [newStates] = droneModel(currentStates,inputs,timeStep)
global i GT step

if i == 1
    GT.X(i) = currentStates(1);
    GT.Y(i) = currentStates(2);
    GT.Z(i) = currentStates(3);
    GT.VX(i) = currentStates(4);
    GT.VY(i) = currentStates(5);
    GT.VZ(i) = currentStates(6);
    GT.PHI(i) = currentStates(7);
    GT.THETA(i) = currentStates(9);
    GT.PSI(i) = currentStates(11);
    GT.P(i) = 0;
    GT.Q(i) = 0;
    GT.R(i) = 0;
    GT.AX(i) = 0;
    GT.AY(i) = 0;
    GT.AZ(i) = 0;
end

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

phi = newStates(7);
theta = newStates(9);
psi = newStates(11);
R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
R_B_E = R_E_B';
v = newStates(4:6)';
%specificForce = R_E_B*(R_B_E*[0 0 T]'+R_B_E*dragMatrix*R_E_B*v);
specificForce = [0 0 T]'+dragMatrix*R_E_B*v;
%specificForce = -R_E_B*[0 0 9.8]';
dAngularVelocity = [newStates(8) newStates(10) newStates(12)]';
 angularVelocity = [1 0 -sin(theta);...
                0 cos(phi) cos(theta)*sin(phi);...
                0 -sin(phi) cos(theta)*cos(phi)]*dAngularVelocity;

    GT.X(i+1) = newStates(1);
    GT.Y(i+1) = newStates(2);
    GT.Z(i+1) = newStates(3);
    GT.VX(i+1) = newStates(4);
    GT.VY(i+1) = newStates(5);
    GT.VZ(i+1) = newStates(6);
    GT.PHI(i+1) = newStates(7);
    GT.THETA(i+1) = newStates(9);
    GT.PSI(i+1) = newStates(11);
    GT.TIME(i+1) = step*(i+1);
    GT.AX(i+1) = specificForce(1);
    GT.AY(i+1) = specificForce(2);
    GT.AZ(i+1) = specificForce(3);
    GT.P(i+1) = angularVelocity(1);
    GT.Q(i+1) = angularVelocity(2);
    GT.R(i+1) = angularVelocity(3);
end