function [ pm, pcm] = computeFK(theta)
% Compute Forward Kinematics for a five link robot 
% pm  is a 2 x 5 array with the joint cartesian position of each body 
% pcm is a 2 x 5 array with the Center of Mass position of each body
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

% global orientations
o1 =  -theta(1);
o2 =  -theta(1) - theta(2);
o3 = -theta(1) - theta(2) - theta(3);
o4 = -theta(1) - theta(2) - theta(3) + theta(4);
o5 = -theta(1) - theta(2) -theta(3) +theta(4) - theta(5);  


% global cartesian positions
pm1 = [ -L1 * sin(o1); L1* cos(o1)];
pm2 = pm1 + [-L2 * sin(o2); L2 *cos(o2)];
pm3 = pm2 + [-L3 * sin(o3); L3 * cos(o3)];
pm4 = pm2 + [L4 * sin(o4); -L4 * cos(o4)];
pm5 = pm4 + [L5 * sin(o5); -L5 * cos(o5)]; 
pm = [pm1 pm2 pm3 pm4 pm5];

% global center of mass positions
pcm1 = [ (-L1 + r1) * sin(o1); (L1 - r1)* cos(o1)];
pcm2 = pm1 + [(-L2 + r2) * sin(o2); (L2-r2) *cos(o2)];
pcm3 = pm2 + [-r3 * sin(o3); r3 * cos(o3)];
pcm4 = pm2 + [r4 * sin(o4); -r4 * cos(o4)];
pcm5 = pm4 + [r5 * sin(o5); -r5 * cos(o5)]; 
pcm = [pcm1 pcm2 pcm3 pcm4 pcm5];


end

