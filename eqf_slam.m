classdef eqf_slam < handle
    
    properties
        X_rb = eye(4); % Observer state
        X_lm = zeros(3,20);
        X_rb_no_inno = eye(4);
        init_lm = transpose([5.66    0.33   -0.66;...
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
                            4.51    0.79    4.51]);
        
        
%         init_lm = transpose([5,2,1; 5,3,4; 5.5,1,3; 5.2,2.5,2; 5.1,2,2.5; 5,1.5,1.5; 5.2,3.1,1.3; 5.2,2,1.8; 5.2,2,2; 4.5,2,2.5; 5,2,1.5; 5,2.5,3.5; 5.3,1.2,2.8; 5.6,2.2,3.1; 4.8,2.5,4; 4.2,2.1,1.8; 4.5,1.8,2; 5.4,1.9,2.0; 5.6,1.2,1.3; 5.7,2.4,2]);
        init_rb = [1,0,0,-1;0,1,0,0;0,0,1,8;0,0,0,1];

        Sigma = eye(60)*3;
        robot_trail = NaN(3,600);
        robot_trail_no_inno = NaN(3,2000);
        
        fx_l = 458.654;
        fy_l = 457.296;
        cx_l = 367.215;
        cy_l = 248.375;
        
        fx_r = 457.587;
        fy_r = 456.134;
        cx_r = 379.999;
        cy_r = 255.238;
        
        xl = [1,0,0,-0.021;0,1,0,-0.06;0,0,1,0.01;0,0,0,1];
        xr = [1,0,0,-0.019;0,1,0,0.04;0,0,1,0.007;0,0,0,1];
        
        
        P = eye(60)*1;
        Q = eye(80)*0.3;
        
        dt = 0.1;
        
        noiseP = 1;
        noisep = 1.5;
        
    end
    
    methods
        
        function obj = eqf_slam()
            obj.init_lm = obj.init_lm + obj.noisep*rand(3,20);
%             obj.init_rb(1:3,4) = obj.init_rb(1:3,4) + obj.noiseP*rand(3,1);
            obj.init_rb(1:3,4) = obj.init_rb(1:3,4) + 2*ones(3,1);
            obj.robot_trail(:,end) = [obj.init_rb(1,4);obj.init_rb(2,4);obj.init_rb(3,4)];
            obj.robot_trail_no_inno(:,end) = [obj.init_rb(1,4);obj.init_rb(2,4);obj.init_rb(3,4)];
        end
        
        
        function update_vel(obj, vel)
              obj.X_rb = obj.X_rb*vel;  
              obj.X_rb_no_inno = obj.X_rb_no_inno*vel;
        end
        
        function update_delta(obj,delta)
            for i=1:20
                d = delta(1+3*(i-1):3*i);
                
                obj.X_lm(:,i) = obj.dt*d+obj.X_lm(:,i);
            end
        end
        
        function [ci,error] = compute_C_i(obj,measurement_l, measurement_r,idx)
            p_hat = obj.init_rb*obj.X_rb;
            
            ci = zeros(80,60);
            error = zeros(80,1);
            for i = 1:20
                pi_hat = obj.init_lm(:,i)+obj.init_rb(1:3,1:3)*obj.X_lm(:,i);
                pi_hat = [pi_hat;1];
                pi_hat_l = inv(p_hat)*pi_hat;
                yi_hat = inv(obj.xl)*pi_hat_l;
                de_l = [obj.fx_l/yi_hat(3), 0, -obj.fx_l*yi_hat(1)/(yi_hat(3)^2); 0, obj.fy_l/yi_hat(3), -obj.fy_l*yi_hat(2)/(yi_hat(3)^2)];
                ci_l = de_l*inv(obj.xl(1:3,1:3))*transpose(obj.X_rb(1:3,1:3));
                
                pi_hat_r = inv(p_hat)*pi_hat;
                yi_hat_r = inv(obj.xr)*pi_hat_r;
                de_r = [obj.fx_r/yi_hat_r(3), 0, -obj.fx_r*yi_hat_r(1)/(yi_hat_r(3)^2); 0, obj.fy_r/yi_hat_r(3), -obj.fy_r*yi_hat_r(2)/(yi_hat_r(3)^2)];
                ci_r = de_r*inv(obj.xr(1:3,1:3))*transpose(obj.X_rb(1:3,1:3));
                
                ci(1+4*(i-1):2+4*(i-1),1+3*(i-1):3+3*(i-1)) = ci_l;
                ci(3+4*(i-1):4+4*(i-1),1+3*(i-1):3+3*(i-1)) = ci_r;
                
                
                    
                
                p_x_l = obj.fx_l*yi_hat(1)/yi_hat(3)+obj.cx_l;
                p_x_r = obj.fx_r*yi_hat_r(1)/yi_hat_r(3)+obj.cx_r;
                p_y_l = obj.fy_l*yi_hat(2)/yi_hat(3)+obj.cy_l;
                p_y_r = obj.fy_r*yi_hat_r(2)/yi_hat_r(3)+obj.cy_r;
                
                if ismember(i,idx)
                    error(1+4*(i-1):2+4*(i-1)) = [measurement_l(:,idx==i)-[p_x_l;p_y_l]];
                    error(3+4*(i-1):4+4*(i-1)) = [measurement_r(:,idx==i)-[p_x_r;p_y_r]];
                end
                    
                    
            end 
        end
        
        function update_Sigma(obj,c)
%             dsigma = obj.P-obj.Sigma*transpose(c)*inv(obj.Q)*c*obj.Sigma;
%             obj.Sigma = obj.Sigma + obj.dt*dsigma;
            
            s = c*obj.Sigma*c'+obj.Q;            
            obj.Sigma = obj.Sigma+obj.dt*(obj.P-obj.Sigma*c'*inv(s)*c*obj.Sigma);
        end
        

        function [Delta,delta] = compute_innovation(obj,c,error)
            s = c*obj.Sigma*c'+obj.Q;
            g = obj.Sigma*transpose(c)*inv(s)*error;
%              g = obj.Sigma*transpose(c)*inv(obj.Q)*error;
            
            R_p0 = obj.init_rb(1:3,1:3);
            A = zeros(60,6);
            for i = 1:20
                q_i = inv(obj.init_rb)*[obj.init_lm(:,i);1];
                qa = skew(q_i(1:3)+obj.X_lm(:,i));
                A(1+3*(i-1):3*i,:) = [R_p0*qa, -R_p0];                
            end
            
            b = g;
            v = inv(A'*A)*A'*b;
            Delta = [skew(v(1:3)),v(4:6);0,0,0,0];
            
            delta=zeros(60,1);
            
            for i = 1:20
                q_i = inv(obj.init_rb)*[obj.init_lm(:,i);1];
                delta(1+3*(i-1):3*i) = g(1+3*(i-1):3*i)+skew(v(1:3))*q_i(1:3)+v(4:6);
                
                
                
            end

        end
        
        function update_innovation(obj,delta)
            
            
            
            obj.X_rb = expm(obj.dt*delta)*obj.X_rb;
            for i = 1:20
                obj.X_lm(:,i) = obj.X_lm(:,i) + obj.dt*delta(1:3,1:3)*obj.X_lm(:,i);
            end
        end
        
        
        
        function [p_est,p_se3,p_no_inno] = output_robot(obj)
            p = obj.init_rb*obj.X_rb;
            p_se3 = p;
            obj.robot_trail(:,1:end-1) = obj.robot_trail(:,2:end);
            obj.robot_trail(:,end) = [p(1,4);p(2,4);p(3,4)];
            p_est = obj.robot_trail;
            
            p_no = obj.init_rb*obj.X_rb_no_inno;
            
            obj.robot_trail_no_inno(:,1:end-1) = obj.robot_trail_no_inno(:,2:end);
            obj.robot_trail_no_inno(:,end) = [p_no(1,4);p_no(2,4);p_no(3,4)];
            p_no_inno = obj.robot_trail_no_inno;
        end
        
        function lm = output_landmarks(obj)
            for i = 1:20
                lm(:,i) =  obj.init_lm(:,i)+obj.init_rb(1:3,1:3)*obj.X_lm(:,i);
            end
            
        end
        
        
        function [lyapn,err] = compute_lyap(obj,P,p)
            A_inv = inv(obj.X_rb);
            PA = P*A_inv;
            err = zeros(60,1);
            
            for i = 1:20
                q_i = inv(obj.init_rb)*[obj.init_lm(:,i);1];
                lm_inv = -A_inv(1:3,1:3)*obj.X_lm(:,i);
                
                lm_new = p(:,i)+P(1:3,1:3)*lm_inv;
                
                q_e = inv(PA)*[lm_new;1];
                
                e = q_e-q_i;
                err(1+3*(i-1):3*i) = e(1:3);
            end
            
            lyapn = err'*inv(obj.Sigma)*err;
            
            
            
        end
        
    
        
    end
    
   
    
end

 function x_skew = skew(x)
    x_skew = [0,-x(3),x(2);...
        x(3),0,-x(1);...
        -x(2),x(1),0];
 end

