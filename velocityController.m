function [attitude_cmd,thrust_cmd] = velocityController(V_r,V_m,psi_m)
global previous_error_v_x previous_error_v_y previous_error_v_z
global sum_error_v_x sum_error_v_y sum_error_v_z


K_P_THETA = -0.5;
K_D_THETA = -0.05;
K_I_THETA = -0.2;

K_P_PHI = 0.5;
K_D_PHI = 0.05;
K_I_PHI = 0.2;

K_D_THRUST = 3.0;
K_P_THRUST = 3.0;
K_I_THRUST = 2.0;
g = 9.8;

R = [cos(psi_m) sin(psi_m) 0;...
    -sin(psi_m) cos(psi_m) 0;...
    0 0 1];
deltaV = R*(V_r-V_m)';

sum_error_v_x = sum_error_v_x + deltaV(1)*0.01;
sum_error_v_y = sum_error_v_y + deltaV(2)*0.01;
sum_error_v_z = sum_error_v_z + deltaV(3)*0.01;
theta_cmd = K_P_THETA*deltaV(1)+K_D_THETA*(deltaV(1)-previous_error_v_x)*100+K_I_THETA*sum_error_v_x;
phi_cmd = K_P_PHI*deltaV(2)+K_D_PHI*(deltaV(2)-previous_error_v_y)*100+K_I_PHI*sum_error_v_y;
thrust_cmd = -g + deltaV(3)*K_P_THRUST+K_I_THRUST*sum_error_v_z;

theta_cmd = bound(theta_cmd);
phi_cmd = bound(phi_cmd);

previous_error_v_x = deltaV(1);
previous_error_v_y = deltaV(2);
previous_error_v_z = deltaV(3);

[attitude_cmd] = [phi_cmd theta_cmd];

end

function [attitude] = bound(unboundAngle)
if unboundAngle > 30/180*pi
    attitude = 30/180*pi;
elseif unboundAngle < -30/180*pi
   attitude = -30/180*pi;
else
    attitude = unboundAngle;
end
    
end