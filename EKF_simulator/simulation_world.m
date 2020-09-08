classdef simulation_world < handle
    %SIMULATION_WORLD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        n = 20;
        landmarks;
        robot = zeros(3,1);
        range = 2;
        drive_cmd = [10; 2*pi] / 100;
        dt = 0.5;
        
        % Noise settings
        lin_vel_noise = 0.1;
        ang_vel_noise = 0.05;
        measurement_noise = 0.01;
    end
    
    methods
        function obj = simulation_world(number)
            obj.n = number;
            
            % generate landmarks 
            radius = obj.drive_cmd(1) / obj.drive_cmd(2);
            centre = [0; radius];
            angles = rand(number,1) * 2*pi;
            dists = (2*rand(number,1)-1)*obj.range + radius;
%             dists = dists + sign(dists)*
            obj.landmarks = centre' + [cos(angles), sin(angles)].*dists;
        end
        
        function [u,q,dt] = drive_robot(obj)
            % Integrate the robot kinematics
            new_robot = obj.robot;
            new_robot(3) = obj.robot(3) + obj.dt*obj.drive_cmd(2);
            new_robot(1) = obj.robot(1) + obj.drive_cmd(1)/obj.drive_cmd(2) * (sin(new_robot(3)) -sin(obj.robot(3)));
            new_robot(2) = obj.robot(2) - obj.drive_cmd(1)/obj.drive_cmd(2) * (cos(new_robot(3)) -cos(obj.robot(3)));
            obj.robot = new_robot;
            
            % Return noisy velocities
            u = obj.drive_cmd(1) + obj.lin_vel_noise^2*randn();
            q = obj.drive_cmd(2) + obj.ang_vel_noise^2*randn();
            dt = obj.dt;
        end
        
        function [measurements, seen_idx] = make_measurement(obj)
            % Measure landmarks
            Rtheta = [cos(obj.robot(3)), -sin(obj.robot(3)); sin(obj.robot(3)), cos(obj.robot(3))];
            measurements = [];
            seen_idx = [];
            for i = 1:size(obj.landmarks,1)
                % Convert the landmark to body frame.
                lm = obj.landmarks(i,:)';
                lm = Rtheta'*(lm - obj.robot(1:2));
                dist = norm(lm);
                % Skip landmarks that are out of range or behind the robot.
                if dist > obj.range || lm(1) < 0
                    continue
                end
                
                measurements = [measurements, lm];
                seen_idx = [seen_idx, i];
            end
            % Add noise to the measurements
            measurements = measurements + obj.measurement_noise^2 * randn(size(measurements));
        end
        
        function draw(obj)
            % Draw the world
            
            % landmarks
            plot(obj.landmarks(:,1), obj.landmarks(:,2),'k.');
            
            % robot
            hold on
            quiver(obj.robot(1), obj.robot(2), 0.5*cos(obj.robot(3)), 0.5*sin(obj.robot(3)), 0, 'b');
            
            % visibility lines
            [~, seen] = obj.make_measurement();
            for i = seen
                plot([obj.robot(1), obj.landmarks(i,1)],[obj.robot(2), obj.landmarks(i,2)], 'y-');
            end
            
            % landmark labels
            for i = 1:obj.n
                text(obj.landmarks(i,1), obj.landmarks(i,2), [" ", num2str(i)]);
            end
            hold off
            
            
            % Axes, labels, title
            radius = obj.drive_cmd(1) / obj.drive_cmd(2);
            xlim([-1,5]);
            ylim([-1,5]); 
            zlim([-1,5]); 
%             axis([-radius-obj.range radius+obj.range -obj.range 2*radius+obj.range]);
            title("Simulation world");
            xlabel("x (m)");
            ylabel("y (m)");
            zlabel("z (m)");
        end
    end
end

