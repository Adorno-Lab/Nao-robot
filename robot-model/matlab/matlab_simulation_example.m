% Example of using the DQ_kinematics objects of the NAO robot. It creates
% an object for each chain, and plots them all together. The arms 
% configurations change from the initial configuration (pointing forwards)
% to both pointing upwards.

close all
clear all
clc

% Create a new DQ_kinematics objects
topcamera = TopCamera.kinematics();
topcamera.name = 'TC';
q_topcamera = [0; 0];

bottomcamera = BottomCamera.kinematics();
bottomcamera.name = 'BC';
q_bottomcamera = [0; 0];

leftarm = LeftArm.kinematics();
leftarm.name = 'LA';
q_leftarm = [0; 0; 0; 0; 0];

rightarm = RightArm.kinematics();
rightarm.name = 'RA';
q_rightarm = [0; 0; 0; 0; 0];

leftleg = LeftLeg.kinematics();
leftleg.name = 'LL';
q_leftleg = [0; 0; 0; 0; 0; 0];

rightleg = RightLeg.kinematics();
rightleg.name = 'RL';
q_rightleg = [0; 0; 0; 0; 0; 0];

% Prepare the visualization
figure;
axis equal;
grid off;
view(55,16);
zoom(1.0);
hold on;

plot(topcamera, q_topcamera);
hold on
plot(bottomcamera, q_bottomcamera);
hold on
plot(leftarm, q_leftarm);
hold on
plot(rightarm, q_rightarm);
hold on
plot(leftleg, q_leftleg);
hold on
plot(rightleg, q_rightleg);

% Define target angles for the arms chains (both arms up)
q_target_leftarm = [-pi/2; 0; 0; 0; 0];
q_target_rightarm = [-pi/2; 0; 0; 0; 0];

% 10 steps to reach targets
i = 0;
while i < 10
    q_leftarm = i*q_target_leftarm/10;
    q_rightarm = i*q_target_rightarm/10;

    pause(0.1);
    i = i + 1;
    plot(topcamera, q_topcamera);
    hold on
    plot(bottomcamera, q_bottomcamera);
    hold on
    plot(leftarm, q_leftarm);
    hold on
    plot(rightarm, q_rightarm);
    hold on
    plot(leftleg, q_leftleg);
    hold on
    plot(rightleg, q_rightleg);
end

