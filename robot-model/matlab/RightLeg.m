classdef RightLeg
    methods (Static)
        function ret = kinematics()
            % Parameters
            hip_offset_z = 0.085;
            hip_offset_y = 0.050;
            thigh_length = 0.100;
            tibia_length = 0.1029;
            foot_height = 0.04519;

            % DH parameters
            DH_theta = [pi/2, pi/4, 0, 0, 0, 0];
            DH_d = [0, 0, 0, 0, 0, 0];
            DH_a = [0, 0, -thigh_length, -tibia_length, 0, 0];
            DH_alpha = [pi/2, pi/2, 0, 0, -pi/2, 0];
            DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,6);
            DH_matrix = [DH_theta; DH_d; DH_a; DH_alpha; DH_type];
            
            obj = DQ_SerialManipulatorDH(DH_matrix);

            % Setting base and reference to the torso
            tyz = 1 + DQ.E * (1/2) * DQ([0, -hip_offset_y, -hip_offset_z]);
            rx = cos((pi/4)/2) + DQ.i * sin((pi/4)/2);
            obj.set_base_frame(tyz * rx);
            obj.set_reference_frame(tyz * rx);

            % Setting end effector
            ry = cos((pi/2)/2) + DQ.j * sin((pi/2)/2);
            rz = cos(pi/2) + DQ.k * sin(pi/2);
            tz = 1 + DQ.E * (1/2) * DQ([0, 0, -foot_height]);
            obj.set_effector(ry * rz * tz);

            ret = obj;
        end

        function ret = get_joints_names()
            %  Return the name of the joints in the chain.
            ret = {'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'};
        end

        function ret = get_joints_limits(current_angle)
            % Return the limits of the joints in the chain.
            %
            % Parameters:
            %     current_angle: current RAnklePitch value in radians (default: 0)
            
            if nargin == 0
                current_angle = 0.0;
            end

            tolerance = 0.1;
  
            % Anti collision limitation: RAnkleRoll limits depend on RAnklePitch value
            if abs(current_angle - pi * (-68.15) / 180) < tolerance
                variable_limits = [-4.30, 2.86];
            elseif abs(current_angle - pi * (-48.13) / 180) < tolerance
                variable_limits = [-9.74, 10.31];
            elseif abs(current_angle - pi * (-40.11) / 180) < tolerance
                variable_limits = [-12.61, 22.80];
            elseif abs(current_angle - pi * (-25.78) / 180) < tolerance
                variable_limits = [-44.06, 22.80];
            elseif abs(current_angle - pi * 5.73 / 180) < tolerance
                variable_limits = [-44.06, 22.80];
            elseif abs(current_angle - pi * 20.05 / 180) < tolerance
                variable_limits = [-31.54, 22.80];
            elseif abs(current_angle - pi * 52.87 / 180) < tolerance
                variable_limits = [2.86, 0.0];
            else
                variable_limits = [-44.06, 22.80];
            end

            % Lower and upper limits in degrees
            lower = [-65.62, ... % RHipYawPitch
                     -45.29, ... % RHipRoll
                     -88, ... % RHipPitch
                     -5.90, ... % RKneePitch
                     -67.97, ... % RAnklePitch
                     variable_limits(1)]; % RAnkleRoll
            upper = [42.44, ... % RHipYawPitch
                     21.74, ... % RHipRoll
                     27.73, ... % RHipPitch
                     121.47, ... % RKneePitch
                     53.40, ... % RAnklePitch
                     variable_limits(2)]; % RAnkleRoll
            limits = [lower, upper];

            % Converting to radians
            ret = pi * limits / 180;
        end
    end
end