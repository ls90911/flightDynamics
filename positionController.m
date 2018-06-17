function [V_r] = positionController(X_r,X_m)
% X_r is reference X_m is measured states
K_P_X = 2;
K_P_Y = 2;
K_P_Z = 1.0;

v_x_r = (X_r(1)-X_m(1))*K_P_X;
v_y_r = (X_r(2)-X_m(2))*K_P_Y;
v_z_r = (X_r(3)-X_m(3))*K_P_Z;

V_r = [v_x_r v_y_r v_z_r];

end