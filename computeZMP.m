function zmp = computeZMP(theta_pos,theta_vel,theta_ac)
% Compute Zero Moment Point by Forward Kinematics
% Paper: ZERO-MOMENT POINT METHOD FOR STABLE BIPED WALKING. Author: M.H.P. Dekker. Eindhoven, July 2009 DCT no.: 2009.072. Page 12. 
% Last modification: 30/03/2014 (by Feliphe G. Galiza)

% parameters according to human body
height = 1.5; % [m]
L1 = 0.285*height; % [m]
L2 = (0.53 - 0.285)*height; % [m]
L3 = height - (L1 + L2); % [m]
L4 = L2; % [m]
L5 =L1; % [m]

r1 = L1/2; % [m]
r2 = L2/2; % [m]
r3 = L3/2; % [m]    
r4 = L4/2; % [m]
r5 = L5/2; % [m]

mass = 50; % [kg]
m1 = 0.061 * mass; % [kg]
m2 = 0.1 * mass; % [kg]
m3 = 0.678 * mass; % [kg]
m4 = m2; % [kg]
m5 = m1; % [kg]

Izz_1 = m1 * 0.735^2; % [kg*m^2]
Izz_2 = m2 * 0.540^2; % [kg*m^2]
Izz_3 = m3 * 0.0798^2; % [kg*m^2]
Izz_4 = Izz_2; % [kg*m^2]
Izz_5 = Izz_1; % [kg*m^2]

% joint angles
theta1 = theta_pos(1);
theta2 = theta_pos(2);
theta3= theta_pos(3);
theta4 = theta_pos(4);
theta5 = theta_pos(5);

% joint velocities
theta1_dot = theta_vel(1);
theta2_dot = theta_vel(2);
theta3_dot = theta_vel(3);
theta4_dot  = theta_vel(4);
theta5_dot = theta_vel(5);

% joint accelerations
theta1_acc = theta_ac(1);
theta2_acc = theta_ac(2);
theta3_acc = theta_ac(3);
theta4_acc  = theta_ac(4);
theta5_acc = theta_ac(5);

zmp =-((981*m5*(r5*sin(theta1 + theta2 + theta3 - theta4 + theta5) - L2*sin(theta1 + theta2) - L1*sin(theta1) + L4*sin(theta1 + theta2 + theta3 - theta4)))/100 + Izz_5*(theta1_acc + theta2_acc + theta3_acc - theta4_acc + theta5_acc))/(m5*((171*cos(theta1 + theta2 + theta3 - theta4 + theta5)*(theta1_dot + theta2_dot + theta3_dot - theta4_dot + theta5_dot)^2)/800 - (171*theta1_dot^2*cos(theta1))/400 + (147*cos(theta1 + theta2 + theta3 - theta4)*(theta1_dot + theta2_dot + theta3_dot - theta4_dot)^2)/400 - (147*sin(theta1 + theta2)*(theta1_acc + theta2_acc))/400 - (147*cos(theta1 + theta2)*(theta1_dot + theta2_dot)^2)/400 - (171*theta1_acc*sin(theta1))/400 + (171*sin(theta1 + theta2 + theta3 - theta4 + theta5)*(theta1_acc + theta2_acc + theta3_acc - theta4_acc + theta5_acc))/800 + (147*sin(theta1 + theta2 + theta3 - theta4)*(theta1_acc + theta2_acc + theta3_acc - theta4_acc))/400 + 981/100));

end


% % Computing ZMP
% 	
% 
% y_acc = formula(diff(eval(pcm(2,:)),t,2)); % acceleration in y direction
% 
% g = 9.81; % gravity [m/s^2]
% 
% for i=1:5
%     num = m(i) * pcm(1,i) * g + I(3,3,i) * w_dot(i); % zmp numerator
%     den = m(i) * (y_acc(i) + g); % zmp denominator
% end
% zmp=num/den;
% 
% % Dependendo da Versão do MATLAB, ele utiliza diff(theta1(t),t) ou D(theta1(t)
% %zmp = subs(zmp,{'diff(theta1(t), t)', 'diff(theta2(t), t)','diff(theta3(t), t)','diff(theta4(t), t)','diff(theta5(t), t)'},{theta_dot.'});
% %zmp = subs(zmp,{'diff(theta1(t), t,t)', 'diff(theta2(t), t,t)','diff(theta3(t), t,t)','diff(theta4(t), t,t)','diff(theta5(t), t,t)'},{theta_acc.'})
% 
% zmp = subs(zmp,{'theta1(t)', 'theta2(t)','theta3(t)','theta4(t)','theta5(t)'},{theta.'});
% zmp = subs(zmp,{'D(theta1)(t)', 'D(theta2)(t)','D(theta3)(t)','D(theta4)(t)','D(theta5)(t)'},{theta_dot.'});
% zmp = subs(zmp,{'D(D(theta1))(t)', 'D(D(theta2))(t)','D(D(theta3))(t)','D(D(theta4))(t)','D(D(theta5))(t)'},{theta_acc.'});