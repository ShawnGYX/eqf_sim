close all
world = simulate(20);

EQF = eqf_slam();

lyapn = NaN(1500,1);
local_err = NaN(1500,60);
for simulation_step = 1:600
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
    
    [trail,pose,trail_no_inno] = EQF.output_robot();
    
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
    

   
    
    % This command draws the world to help you visualise what's going on.
    % You could try and plot your EKF output on top to compare.
    world.draw(trail,pose,unaligned_landmark,trail_no_inno);
    drawnow
end