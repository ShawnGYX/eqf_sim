close all
world = simulate(20);

s_1=[];
s_2=[];

I4 = eye(4);

len = 210;

EQF = eqf_slam();

lm_trail = NaN(3,len,6);

lyapn = NaN(len,1);
local_err = NaN(len,60);
for simulation_step = 1:210
    % The robot drives in the simulated world. The simulator works out the
    % forward kinematics of the robot and provides you with noisy velocity
    % estimates. In practice, you would get these from your inputs to the
    % robot. You can edit the simulation_world class to change the noise
    % settings.
    [u,dt] = world.drive_robot();
    % The simulator provides measurements of nearby landmarks. In practice,
    % you would use ARUCO detection for this.
    % You use the ARUCO system to measure landmark positions.
    [measurements_l,measurements_r, idx] = world.make_measurement();
    if length(idx)<20
        printf(idx);
    end
    
    EQF.update_vel(u);
    
    
    [c,err] = EQF.compute_C_i(measurements_l,measurements_r,idx);
    
    EQF.update_Sigma(c);
    
%     d = EQF.compute_Delta(c,err);
    
    
    
   [Delta,delta] = EQF.compute_innovation(c,err);
   
    EQF.update_delta(delta);
    
    EQF.update_innovation(Delta);
    
    [trail,pose,trail_no_inno,p_no] = EQF.output_robot();
    
    s1=pose*inv(world.robot)-I4;
    s2=p_no*inv(world.robot)-I4;
    s_1=[s_1;norm(s1)];
    s_2=[s_2;norm(s2)];
    [lyapn(simulation_step),local_err(simulation_step,:)] = EQF.compute_lyap(world.robot,world.landmarks');
    
    s = pose*inv(world.robot);
    
    unaligned_landmark = EQF.output_landmarks();
    
    landmarks = EQF.X_lm;
    lm = NaN(4,20);
    for i = 1:20
        pi_hat = EQF.init_lm(:,i)+EQF.init_rb(1:3,1:3)*landmarks(:,i);
        lm(:,i) = inv(s)*[pi_hat;1];
    end
    aligned_lm = lm(1:3,:)';
    
    ind = [1,7,9,11,13,17];
    
    for i =1:6
        lm_trail(:,1:end-1,i) = lm_trail(:,2:end,i);
        lm_trail(:,end,i) = unaligned_landmark(:,ind(i));
    end
%     lm_trail(simulation_step,:,1) = unaligned_landmark(:,1)';
%     lm_trail(simulation_step,:,2) = unaligned_landmark(:,7)';
%     lm_trail(simulation_step,:,3) = unaligned_landmark(:,9)';
%     lm_trail(simulation_step,:,4) = unaligned_landmark(:,11)';
%     lm_trail(simulation_step,:,5) = unaligned_landmark(:,13)';
%     lm_trail(simulation_step,:,6) = unaligned_landmark(:,17)';
   
    
    % This command draws the world to help you visualise what's going on.
    % You could try and plot your EKF output on top to compare.
    world.draw(trail,pose,unaligned_landmark,trail_no_inno,NaN(3,600,6));
    drawnow
end

for j = 1:len
    
    for i =1:6
        
        lm_i = inv(s)*[lm_trail(:,j,i);1];
        lm_trail(:,j,i) = lm_i(1:3);
        
    end
    t = inv(s)*[trail(:,j);1];
        
        trail(:,j) = t(1:3);
    
    
end

pose = inv(s)*pose;
world.draw(trail,pose,aligned_lm',trail_no_inno,lm_trail);
drawnow

