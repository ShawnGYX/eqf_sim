classdef simulate < handle
    %SIMULATION_WORLD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        n = 20;
        
        ang_v = -2.5;
        lin_v = -0.2;
        
        fx_l = 458.654;
        fy_l = 457.296;
        cx_l = 367.215;
        cy_l = 248.375;
        
        fx_r = 457.587;
        fy_r = 456.134;
        cx_r = 379.999;
        cy_r = 255.238;
        
        
        landmarks;
        robot = [1,0,0,-1;0,1,0,0;0,0,1,8;0,0,0,1];
        observer;
        trail = [];
        trailline;
        

        dt = 0.1;
        
        % Noise settings
        lin_vel_noise = 0.1*0.;
        ang_vel_noise = 0.05*0.;
        measurement_noise = 0.31*0;
    end
    
    methods
        function obj = simulate(number)
            obj.n = number;
            
            % generate landmarks 
            
            obj.trail = NaN(3,2000);
            obj.trail(:,end) = [obj.robot(1,4);obj.robot(2,4);obj.robot(3,4)];   
            
            obj.landmarks = [5.66    0.33   -0.66;...
                            5.29   -3.40    4.98;...
                            5.32   -0.98    2.88;...
                            5.01   -0.24    2.06;...
                            5.98    3.57    1.55;...
                            4.10    2.03    1.52;...
                            5.70    2.79   -1.35;...
                            4.43   -3.40    2.43;...
                            4.84    0.06    4.21;...
                            4.78   -2.18    1.95;...
                            5.39    3.67    4.43;...
                            5.77    0.69    3.94;...
                            4.53   -3.69    1.26;...
                            5.51    0.73    0.22;...
                            4.69    1.25    3.49;...
                            5.60   -2.31    6.10;...
                            4.96   -1.49    4.09;...
                            4.38    2.61    3.28;...
                            5.33    2.54    4.63;...
                            4.51    0.79    4.51];
%             obj.landmarks = [5,2,1; 5,3,4; 5.5,1,3; 5.2,2.5,2; 5.1,2,2.5; 5,1.5,1.5; 5.2,3.1,1.3; 5.2,2,1.8; 5.2,2,2; 4.5,2,2.5; 5,2,1.5; 5,2.5,3.5; 5.3,1.2,2.8; 5.6,2.2,3.1; 4.8,2.5,4; 4.2,2.1,1.8; 4.5,1.8,2; 5.4,1.9,2.0; 5.6,1.2,1.3; 5.7,2.4,2];
        end
        
        function [u,dt] = drive_robot(obj)
            % Integrate the robot kinematics
            vel = eye(4);
            vel(1:3,1:3) = angle2dcm(0,0,obj.ang_v/180*pi);
            vel(2,4) = obj.lin_v; 
            
            new_robot = obj.robot*vel;
            obj.robot = new_robot;
            
            obj.trail(:,1:end-1) = obj.trail(:,2:end);
            obj.trail(:,end) = [obj.robot(1,4);obj.robot(2,4);obj.robot(3,4)];
            
            % Return noisy velocities
            u = eye(4);
            u(1:3,1:3) = angle2dcm(0,0,(obj.ang_v+obj.ang_vel_noise*rand(1))/180*pi);
            u(2,4) = obj.lin_v + obj.lin_vel_noise *(rand(1));
%             u(2,4) = u(2,4) + vnoise*(2*rand(1)-1);
%             u(1:3,4) = u(1:3,4)+vnoise*rand(3,1);
            dt = obj.dt;
        end
        
        function [measurements_l,measurements_r, seen_idx] = make_measurement(obj)
            % Measure landmarks
            
            measurements_l = [];
            measurements_r = [];
            
            xl = [1,0,0,-0.021;0,1,0,-0.06;0,0,1,0.01;0,0,0,1];
            xr = [1,0,0,-0.019;0,1,0,0.04;0,0,1,0.007;0,0,0,1];

            seen_idx = [];
            for i = 1:size(obj.landmarks,1)
                % Convert the landmark to body frame.
                lm = [obj.landmarks(i,:)';1];
                lm_l = inv(xl)*inv(obj.robot)*lm;
                lm_r = inv(xr)*inv(obj.robot)*lm;
%                 dist = norm(lm_l);
                % Skip landmarks that are out of range or behind the robot.
%                 if dist > obj.range || lm(1) < 0
                
                p_x_l = obj.fx_l*lm_l(1)/lm_l(3)+obj.cx_l;
                p_x_r = obj.fx_r*lm_r(1)/lm_r(3)+obj.cx_r;
                p_y_l = obj.fy_l*lm_l(2)/lm_l(3)+obj.cy_l;
                p_y_r = obj.fy_r*lm_r(2)/lm_r(3)+obj.cy_r;
%                 if lm_l(1)<0 ||lm_r(1)<0 ||abs(p_x_l)>640 || abs(p_x_r)>640|| abs(p_y_l)>640||abs(p_y_r)>640
                if lm_l(1)<0||lm_r(1)<0                
                    continue
                end
                measurements_l = [measurements_l, [p_x_l;p_y_l]];
                measurements_r = [measurements_r, [p_x_r;p_y_r]];
                seen_idx = [seen_idx, i];
            end
            % Add noise to the measurements
            measurements_l = measurements_l + obj.measurement_noise * (2*rand(size(measurements_l))-ones(size(measurements_l)));
            measurements_r = measurements_r + obj.measurement_noise * (2*rand(size(measurements_l))-ones(size(measurements_l)));
        end
        
        function draw(obj,trail,pose,lm,no_inno,lm_trail)           
            % landmarks
            obj.trailline = plot3(obj.trail(1,:), obj.trail(2,:), obj.trail(3,:),'r', 'LineWidth', 2);
            hold on
             
            
            h3=plot3(trail(1,:), trail(2,:), trail(3,:),'g-.', 'LineWidth', 2);
%             h4=plot3(no_inno(1,:), no_inno(2,:), no_inno(3,:),'k', 'LineWidth', 2);
            drawAxes(pose,[trail(1,end), trail(2,end), trail(3,end)]);
            drawAxes(obj.robot(1:3,1:3),[obj.robot(1,4),obj.robot(2,4),obj.robot(3,4)]);
            ind = [1,7,9,11,13,17];
            for i = 1:6
                h1=plot3(obj.landmarks(ind(i),1), obj.landmarks(ind(i),2),obj.landmarks(ind(i),3),'ko','MarkerSize',10);
                h2=plot3(lm(1,ind(i)),lm(2,ind(i)),lm(3,ind(i)),'color', [0 0.5 0],'Marker','+','MarkerSize',15);
                plot3(lm_trail(1,:,i),lm_trail(2,:,i),lm_trail(3,:,i),'color', [0 0.5 0], 'LineWidth', 2); 
            end
            
            
            
            % robot
            
            p = [obj.robot(1,4),obj.robot(2,4), obj.robot(3,4)];
            plot3(p(1),p(2),p(3),'b.','MarkerSize', 10);

%             legend([h1 h2 obj.trailline h3 h4],'True landmark position','Estimated landmark position','True Robot Trajectory','Estimated Robot Trajectory','Estimated Robot Trajectory without Innovation')
            



% visibility lines
%             [~,~, seen] = obj.make_measurement();
%             for i = seen
%                 plot3([p(1), obj.landmarks(i,1)],[p(2), obj.landmarks(i,2)],[p(3), obj.landmarks(i,3)], 'y-');
%             end
            
%             landmark labels
%             for i = 1:obj.n
%                 text(obj.landmarks(i,1), obj.landmarks(i,2), [" ",
%                 num2str(i)]);
%             end
            hold off
            grid on;
            xlim([-5,8]);
            ylim([-5,6]); 
            zlim([-3,9]); 
%             axis([-radius-obj.range radius+obj.range -obj.range 2*radius+obj.range]);
%             title("EqF Simulation",'FontSize',15);
            xlabel("x (m)");
            ylabel("y (m)");
            zlabel("z (m)");
            
        end
    end
end

