
% This is an attempt to build a controller for a five link robot using the toolbox Adaptive Neuro-Fuzzy training of Sugeno-type FIS a.k.a ANFIS.
% Actually I am trying to implement something like what was developed on this tutorial:
% http://www.mathworks.com/help/fuzzy/examples/modeling-inverse-kinematics-in-a-robotic-arm.html
% Last modification: 30/03/2014 (by Feliphe G. Galiza)

clear
clc

load('min_tf.mat')
% the structure opt is from the Optimal Control solution using PROPT.
% opt.q are the joint angles
% opt.qd are the joint velocites
% opt.u joint torques

q0 = 2*pi*rand(1,10);
[t,X] = integrator(q0,opt);


% opt.u = [opt.u; zeros(11,5)]; % torques

% data1 = [X opt.u(:,1)]; % create x-y-theta1 dataset
% data2 = [X opt.u(:,2)]; % create x-y-theta1 dataset
% data3 = [X opt.u(:,3)]; % create x-y-theta1 dataset
% data4 = [X opt.u(:,4)]; % create x-y-theta1 dataset
% data5 = [X opt.u(:,5)]; % create x-y-theta1 dataset
% 
% fprintf('-->%s\n','Start training first ANFIS network. It may take one minute depending on your computer system.')
% anfis1 = anfis(data1, 7, 150) % train first ANFIS network
% % anfis2 = anfis(data2, 7, 150, [0,0,0,0]); % train first ANFIS network
% % anfis3 = anfis(data3, 7, 150, [0,0,0,0]); % train first ANFIS network
% % anfis4 = anfis(data4, 7, 150, [0,0,0,0]); % train first ANFIS network
% % anfis5 = anfis(data5, 7, 150, [0,0,0,0]); % train first ANFIS network
