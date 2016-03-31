
syms theta1 theta2 theta3 theta4 theta5 theta1_dot theta2_dot theta3_dot theta4_dot theta5_dot theta1_acc theta2_acc theta3_acc theta4_acc theta5_acc m1 m2 m3 m4 m5 m6 r1 r2 r3 r4 r5 L1 L2 L3 L4 L5 Izz_1 Izz_2 Izz_3 Izz_4 Izz_5

% joints position velocities and accelerations
theta = [theta1; theta2; theta3; theta4 ;theta5];
theta_dot =[theta1_dot; theta2_dot; theta3_dot; theta4_dot; theta5_dot];
theta_acc =[theta1_acc; theta2_acc; theta3_acc; theta4_acc; theta5_acc];


% moments of inertia in respect to the base cordinate frame. i.e. Inertial Frame
I(:,:,1) = [0, 0, 0; 0, 0, 0; 0,0, Izz_1];
I(:,:,2) = [0, 0, 0; 0, 0, 0; 0,0, Izz_2];
I(:,:,3) = [0, 0, 0; 0, 0, 0; 0,0, Izz_3];
I(:,:,4) = [0, 0, 0; 0, 0, 0; 0,0, Izz_4];
I(:,:,5) = [0, 0, 0; 0, 0, 0; 0,0, Izz_5];

% orientations
o1 =  -theta(1);
o2 =  -theta(1) - theta(2);
o3 = -theta(1) - theta(2) - theta(3);
o4 = -theta(1) - theta(2) - theta(3) + theta(4);
o5 = -theta(1) - theta(2) -theta(3) +theta(4) - theta(5);  
o =  [o1 o2 o3 o4 o5];


% angular accelerations

pm1 = [ -L1 * sin(o1); L1* cos(o1)];
pm2 = pm1 + [-L2 * sin(o2); L2 *cos(o2)];
pm3 = pm2 + [-L3 * sin(o3); L3 * cos(o3)];
pm4 = pm2 + [L4 * sin(o4); -L4 * cos(o4)];
pm5 = pm4 + [L5 * sin(o5); -L5 * cos(o5)]; 

pcm1 = [ (-L1 + r1) * sin(o1); (L1 - r1)* cos(o1)];
pcm2 = pm1 + [(-L2 + r2) * sin(o2); (L2-r2) *cos(o2)];
pcm3 = pm2 + [-r3 * sin(o3); r3 * cos(o3)];
pcm4 = pm2 + [r4 * sin(o4); -r4 * cos(o4)];
pcm5 = pm4 + [r5 * sin(o5); -r5 * cos(o5)]; 
pcm = [pcm1 pcm2 pcm3 pcm4 pcm5];

mass = 50; % kg
m1 = 0.061 * mass;
m2 = 0.1 * mass;
m3 = 0.678 * mass;
m4 = m2;
m5 = m1;

m = [m1; m2; m3; m4; m5];

% struct to store jacobian matrices
robotBody = struct('Jp',[],'Jo',[],'J',[]);

% building jacobian matrices
for corpo=1:5
    robotBody(corpo).Jp = [jacobian(pcm(:,corpo),theta.');zeros(1,5)];
    robotBody(corpo).Jo = [zeros(2,5);jacobian(o(corpo),theta.')];
    robotBody(corpo).J  = [robotBody(corpo).Jp; robotBody(corpo).Jo];
end

% building global inertia matrix
H =sym(zeros(5,5));
    for corpo=1:5
        H = H + m(corpo)*(  robotBody(corpo).Jp.')*(robotBody(corpo).Jp) + (robotBody(corpo).Jo.')*I(:,:,corpo)*(robotBody(corpo).Jo);
    end
    
% building Christoffel’s Three-Index Symbol
C = sym(zeros(5,5,5));
 for k=1:5
     for i=1:5
         for j=1:5
            C(i,j,k) = diff(H(i,j),theta(k)) - 1/2*diff(H(j,k),theta(i));
         end
     end
 end
     
% building Centrifugal and Coriolis terms    
 h = sym(zeros(5,1));
 for i=1:5
        for j=1:5
            for k=1:5
                h(i) = h(i) + C(i,j,k)*theta_dot(j)*theta_dot(k);
            end
        end
 end
 
 g = 9.81; % gravity [m/s^2]

% building potential energy 
U=0;
for corpo=1:5
    U = U + m(corpo)*g*pcm(2,corpo);
end

% taking the potential energy derivative
dU=sym(zeros(1,5)); 
for corpo=1:5
    dU(corpo) = diff(U,theta(corpo));
end

height = 1.5; % m
L1 = 0.285*height;
L2 = (0.53 - 0.285)*height;
L3 = height - (L1 + L2);
L4 = L2;
L5 =L1;

r1 = L1/2; % m
r2 = L2/2;
r3 = L3/2;
r4 = L4/2;
r5 = L5/2;

mass = 50; % kg
m1 = 0.061 * mass;
m2 = 0.1 * mass;
m3 = 0.678 * mass;
m4 = m2;
m5 = m1;

% using all proximal
Izz_1 = m1 * 0.735^2;
Izz_2 = m2 * 0.540^2;
Izz_3 = m3 * 0.0798^2;
Izz_4 = Izz_2;
Izz_5 = Izz_1;

% moments of inertia in respect to the base cordinate frame. i.e. Inertial Frame
I(:,:,1) = [0, 0, 0; 0, 0, 0; 0,0, Izz_1];
I(:,:,2) = [0, 0, 0; 0, 0, 0; 0,0, Izz_2];
I(:,:,3) = [0, 0, 0; 0, 0, 0; 0,0, Izz_3];
I(:,:,4) = [0, 0, 0; 0, 0, 0; 0,0, Izz_4];
I(:,:,5) = [0, 0, 0; 0, 0, 0; 0,0, Izz_5];


% orientations
o1 =  -theta(1);
o2 =  -theta(1) - theta(2);
o3 = -theta(1) - theta(2) - theta(3);
o4 = -theta(1) - theta(2) - theta(3) + theta(4);
o5 = -theta(1) - theta(2) -theta(3) +theta(4) - theta(5);
o =  [o1 o2 o3 o4 o5];

% angular accelerations

pm1 = [ -L1 * sin(o1); L1* cos(o1)];
pm2 = pm1 + [-L2 * sin(o2); L2 *cos(o2)];
pm3 = pm2 + [-L3 * sin(o3); L3 * cos(o3)];
pm4 = pm2 + [L4 * sin(o4); -L4 * cos(o4)];
pm5 = pm4 + [L5 * sin(o5); -L5 * cos(o5)];

pcm1 = [ (-L1 + r1) * sin(o1); (L1 - r1)* cos(o1)];
pcm2 = pm1 + [(-L2 + r2) * sin(o2); (L2-r2) *cos(o2)];
pcm3 = pm2 + [-r3 * sin(o3); r3 * cos(o3)];
pcm4 = pm2 + [r4 * sin(o4); -r4 * cos(o4)];
pcm5 = pm4 + [r5 * sin(o5); -r5 * cos(o5)];
pcm = [pcm1 pcm2 pcm3 pcm4 pcm5];


