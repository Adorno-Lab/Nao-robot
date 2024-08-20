classdef LeftArm
    methods (Static)
        function ret = kinematics()
            % Parameters
            shoulder_offset_z = 0.100;
            shoulder_offset_y = 0.098;
            elbow_offset_y = 0.015;
            upper_arm_length = 0.105;
            lower_arm_length = 0.05595;
            hand_offset_x = 0.05775;
            hand_offset_z = 0.01231;

            % DH parameters
            DH_theta = [0, pi/2, 0, 0, 0];
            DH_d = [0, 0, upper_arm_length, 0, lower_arm_length];
            DH_a = [0, elbow_offset_y, 0, 0, 0];
            DH_alpha = [pi/2, pi/2, -pi/2, pi/2, 0];
            DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,5);
            DH_matrix = [DH_theta; DH_d; DH_a; DH_alpha; DH_type];
            
            obj = DQ_SerialManipulatorDH(DH_matrix);

            % Setting base and reference to the torso
            tyz = 1 + DQ.E * (1/2) * DQ([0, shoulder_offset_y, shoulder_offset_z]);
            rx = cos((-pi/2)/2) + DQ.i * sin((-pi/2)/2);
            obj.set_base_frame(tyz * rx);
            obj.set_reference_frame(tyz * rx);

            % Setting end effector
            rx = cos((-pi/2)/2) + DQ.i * sin((-pi/2)/2);
            rz = cos((-pi/2)/2) + DQ.k * sin((-pi/2)/2);
            txz = 1 + DQ.E * (1/2) * DQ([hand_offset_x, 0, -hand_offset_z]);
            obj.set_effector(rx * rz * txz)

            ret = obj;
        end

        function ret = get_joints_names()
            %  Return the name of the joints in the chain.
            ret = {'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'};
        end

        function ret = get_joints_limits()
            % Return the limits of the joints in the chain.
           
            % Lower and upper limits in degrees
            lower = [-119.5, ... % LShoulderPitch
                     -18, ... % LShoulderRoll
                     -119.5, ... % LElbowYaw
                     -88.5, ... % LElbowRoll
                     -104.5]; % LWristYaw
            upper = [119.5, ... % LShoulderPitch
                     76, ... % LShoulderRoll
                     119.5, ... % LElbowYaw
                     -2, ... % LElbowRoll
                     104.5]; % LWristYaw
            limits = [lower, upper];

            % Converting to radians
            ret = pi * limits / 180;
        end
    end
end