% Example of simulation of the NAO robot using CoppeliaSim. It creates
% an object for each chain. The arms configurations change from the initial
% configuration (pointing forwards) to both arms pointing upwards.

% Before running:
%   - Open CoppeliaSim
%   - Open the 'nao_scene.ttt' or add a new NAO robot to a blank scene

close all
clear all
clc

%% CoppeliaSim (formerly known as V-REP) setup
% Create a DQ_VrepInterface object and start communication with V-REP
vi = DQ_VrepInterface();

% Finish any previous CoppeliaSim communication
vi.stop_simulation();
vi.disconnect_all();

% Start a new connection
vi.connect('127.0.0.1',19997);
disp('Communication established!')

% Start simulation otherwise child scripts cannot be called
vi.set_synchronous(true);
vi.start_simulation();
disp('Simulation started!')

% Define robot interfaces
topcamera = NaoTopCameraVrepRobot('NAO', vi);
bottomcamera = NaoBottomCameraVrepRobot('NAO', vi);
leftarm = NaoLeftArmVrepRobot('NAO', vi);
rightarm = NaoRightArmVrepRobot('NAO', vi);
leftleg = NaoLeftLegVrepRobot('NAO', vi);
rightleg = NaoRightLegVrepRobot('NAO', vi);

% Define target angles for the arms chains (both arms up)
q_target_leftarm = [-pi/2; 0; 0; 0; 0];
q_target_rightarm = [-pi/2; 0; 0; 0; 0];

% 10 steps to reach targets
i = 0;
while i < 10
    leftarm.set_target_configuration_space_positions(i*q_target_leftarm/10);
    rightarm.set_target_configuration_space_positions(i*q_target_rightarm/10);
    
    pause(0.1);
    i = i + 1;
    vi.trigger_next_simulation_step();
    vi.wait_for_simulation_step_to_end();
end

pause(2);

%% Finishes CoppeliaSim communication
vi.stop_simulation();
disp('Simulation finished!')
vi.disconnect_all();
disp('Communication finished!')