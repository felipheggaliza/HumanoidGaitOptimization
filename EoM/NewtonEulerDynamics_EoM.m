
% Computes the simbolic Equation of Motion of a Five Link Robot using
% Newton-Euler Formulation
% Last modification: 30/03/2014 (by Feliphe G. Galiza)

% Symbolic variables
syms theta1 theta2 theta3 theta4 theta5 theta1_dot theta2_dot theta3_dot theta4_dot theta5_dot theta1_acc theta2_acc theta3_acc theta4_acc theta5_acc
syms u1 u2 u3 u4 u5 % forces and moments
syms m1 m2 m3 m4 m5 r1 r2 r3 r4 r5 L1 L2 L3 L4 L5 J1 J2 J3 J4 J5 g

% Vector of generalized coordinates
q = [theta1; theta2; theta3; theta4; theta5];

% Vector of generalized velocities
qd = [theta1_dot; theta2_dot; theta3_dot; theta4_dot; theta5_dot];

% orientations
o1 =  -q(1);
o2 =  -q(1) - q(2);
o3 = -q(1) - q(2) - q(3);
o4 = -q(1) - q(2) - q(3) + q(4);
o5 = -q(1) - q(2) -q(3) +q(4) - q(5);
o =  [o1 o2 o3 o4 o5];

% position and orientation vector
r(1,1) = (-L1 + r1) * sin(o1); % x cm position of link-1
r(2,1) = (L1 - r1)* cos(o1); % y cm position of link-1
r(3,1) = r(1,1) + (-L2 + r2) * sin(o2);% x cm position of link-2
r(4,1) = r(2,1) + (L2-r2) *cos(o2); % y cm position of link-2
r(5,1) = r(3,1) - r3 * sin(o3);% x cm position of link-3
r(6,1) = r(4,1) + r3 * cos(o3);  % y cm position of link-3
r(7,1) = r(3,1) + r4 * sin(o4); % x cm position of link-4
r(8,1) = r(4,1) - r4 * cos(o4); % y cm position of link-4
r(9,1) = r(7,1) + r5 * sin(o5); % x cm position of link-5
r(10,1) = r(8,1)- r5 * cos(o5); % y cm position of link-5
r(11,1) = o(1); % orientation of link-1 
r(12,1) = o(2); % orientation of link-2 
r(13,1) = o(3); % orientation of link-3 
r(14,1) = o(4); % orientation of link-4 
r(15,1) = o(5); % orientation of link-5 

% Inertia matrix
MM = diag([m1 m1 m2 m2 m3 m3 m4 m4 m5 m5 J1 J2 J3 J4 J5]);

% Vector of applied forces and moments, excluding the reaction forces and
% moments
Fe = [0; -m1*g; 0; -m2*g; 0; -m3*g; 0; -m4*g; 0; -m5*g; -u1+u2; -u2+u3; -u3-u4; u4+u5; -u5];

% Jacobian
J = [diff(r, theta1), diff(r, theta2), diff(r,theta3), diff(r,theta4), diff(r,theta5)];

% Transposed Jacobian
Jt = J.';

% Derivative of Jacobian in time
for j = 1:length(J(1,:))
        Jd(:,j) = [diff(J(:,j),q(1)), diff(J(:,j),q(2)), diff(J(:,j),q(3)),diff(J(:,j),q(4)), diff(J(:,j),q(5))]*qd;
end

% Mass matriz
M = Jt*MM*J;
M = simplify(M)

% Vector of generalized Coriolis, centrifugal and gyrocopic forces
k = Jt*MM*Jd*qd;
k = simplify(k)

% Vector of generalized applied forces
ke = Jt*Fe;
ke = simplify(ke)

% Right-hand-side of equations of motion
rhs = ke - k;
rhs = simplify(rhs)

% Equation of Motion is in the form M*x_dot = rhs, where x_dot is the time
% derivative of the states [q qd]'

