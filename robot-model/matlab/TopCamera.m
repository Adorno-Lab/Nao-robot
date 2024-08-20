classdef TopCamera
    methods (Static)
        function ret = kinematics()
            % Parameters
            neck_offset_z = 0.1265;
            top_camera_x = 0.05871;
            top_camera_z = 0.06364;
            top_camera_wy = 0.0209;

            % DH parameters
            DH_theta = [0, 0];
            DH_d = [0, 0];
            DH_a = [0, 0];
            DH_alpha = [-pi/2, 0];
            DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,2);
            DH_matrix = [DH_theta; DH_d; DH_a; DH_alpha; DH_type];
            
            obj = DQ_SerialManipulatorDH(DH_matrix);

            % Setting base and reference to the torso
            tz = 1 + DQ.E * (1/2) * DQ([0, 0, neck_offset_z]);
            obj.set_base_frame(tz);
            obj.set_reference_frame(tz);

            % Setting end effector
            rx = cos((pi/2)/2) + DQ.i * sin((pi/2)/2);
            txz = 1 + DQ.E * (1/2) * DQ([top_camera_x, 0, top_camera_z]);
            ry = cos(top_camera_wy/2) + DQ.j * sin(top_camera_wy/2);
            obj.set_effector(rx * txz * ry);

            ret = obj;
        end

        function ret = get_joints_names()
            % Return the name of the joints in the chain.
            ret = {'HeadYaw', 'HeadPitch'};
        end

        function ret = get_joints_limits(current_angle)
            % Return the limits of the joints in the chain.
            %
            % Parameters:
            %     current_angle: current HeadYaw value in radians (default: 0)
            
            if nargin == 0
                current_angle = 0.0;
            end

            tolerance = 0.1;
  
            % Anti collision limitation: HeadPitch limits depend on HeadYaw value
            if abs(current_angle - pi*(-119.52)/180) < tolerance
                variable_limits = [-25.73, 18.91];
            elseif abs(current_angle - pi * (-87.49) / 180) < tolerance
                variable_limits = [-18.91, 11.46];
            elseif abs(current_angle - pi * (-62.45) / 180) < tolerance
                variable_limits = [-24.64, 17.19];
            elseif abs(current_angle - pi * (-51.74) / 180) < tolerance
                variable_limits = [-27.50, 18.91];
            elseif abs(current_angle - pi * (-43.32) / 180) < tolerance
                variable_limits = [-31.40, 21.20];
            elseif abs(current_angle - pi * (-27.85) / 180) < tolerance
                variable_limits = [-38.50, 24.18];
            elseif abs(current_angle - pi * 0.0 / 180) < tolerance
                variable_limits = [-38.50, 29.51];
            elseif abs(current_angle - pi * 27.85 / 180) < tolerance
                variable_limits = [-38.50, 24.18];
            elseif abs(current_angle - pi * 43.32 / 180) < tolerance
                variable_limits = [-31.40, 21.20];
            elseif abs(current_angle - pi * 51.74 / 180) < tolerance
                variable_limits = [-27.50, 18.91];
            elseif abs(current_angle - pi * 62.45 / 180) < tolerance
                variable_limits = [-24.64, 17.19];
            elseif abs(current_angle - pi * 87.49 / 180) < tolerance
                variable_limits = [-18.91, 11.46];
            elseif abs(current_angle - pi * 119.52 / 180) < tolerance
                variable_limits = [-25.73, 18.91];
            else
                variable_limits = [-38.5, 29.5];
            end

            % Lower and upper limits in degrees
            lower = [-119.5, ... % HeadYaw
                     variable_limits(1)]; % HeadPitch
            upper = [119.5, ... % HeadYaw
                     variable_limits(2)]; % HeadPitch
            limits = [lower, upper];

            % Converting to radians
            ret = pi * limits / 180;
        end
    end
end