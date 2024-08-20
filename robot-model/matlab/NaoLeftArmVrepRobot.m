classdef NaoLeftArmVrepRobot < DQ_SerialVrepRobot
    methods
        function obj = NaoLeftArmVrepRobot(robot_name, vrep_interface)
            % This method constructs an instance of a NaoLeftArmVrepRobot.
            % Usage:
            %     NaoLeftArmVrepRobot(robot_name, vrep_interface)
            %          robot_name: The name of the robot in the CoppeliaSim scene.
            %          vrep_interface: The DQ_VrepInterface object connected to the CoppeliaSim scene.
            %
            % Example: 
            %      vi = DQ_VrepInterface();
            %      vi.connect('127.0.0.1',19997);
            %      vrep_robot = NaoLeftArmVrepRobot('NAO', vi);
            %     
            %     Note that the name of the robot should be EXACTLY the same as in the CoppeliaSim
            %     scene. For instance, if you drag-and-drop a second robot, its name will become 
            %     "NAO[1]", a third robot, "NAO[2]", and so on, while the first one will become "NAO[0]".

            % Call DQ_SerialVrepRobot constructor with the following parameters:
            %   DQ_SerialVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface);  
            %          base_robot_name: The base name of the robot in the CoppeliaSim scene. (ex: NAO)
            %          robot_dof: The number of DoF of the robot in the CoppeliaSim scene.
            %          robot_name: The name of the robot in the CoppeliaSim scene (ex: NAO[0], NAO[1], etc)
            %          vrep_interface: The DQ_VrepInterface object connected to the CoppeliaSim scene.
            %
            obj@DQ_SerialVrepRobot('NAO', 5, robot_name, vrep_interface);
            
            splited_name = strsplit(robot_name,'[');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'NAO')
                error('Expected NAO robot')
            end
            if length(splited_name) > 1
                splited_name2 = strsplit(splited_name{2},']');
                robot_index = splited_name{1};
            else
                robot_index = '';
            end

            % Initialize joint names and base frame
            joints_names = LeftArm.get_joints_names;
            obj.joint_names = {};
            for i=1:length(joints_names)
                current_joint_name = {'/',robot_name,'/',joints_names{i}};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
        
        end
        
        function kin = kinematics(obj)
            % This method constructs an instance of a DQ_SerialManipulatorDH.
            % Usage:
            %     kinematics();
            %
            % Example: 
            %      vi = DQ_VrepInterface();
            %      vi.connect('127.0.0.1',19997);
            %      vrep_robot = NaoLeftArmVrepRobot('NAO', vi);
            %      robot_kinematics = vrep_robot.kinematics();

            % Create a DQ_SerialManipulatorDH object
            kin = LeftArm.kinematics;
            
            % Update base and reference frame with V-REP values
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
        end
    end
end