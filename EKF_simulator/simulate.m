% The rng command makes sure that matlab uses the same random numbers each
% time. You can comment out this to let matlab generate a different world
% and different noise for each simulation.
rng(0);

% Set up a simulation environment with 20 landmarks
world = simulation_world(20);

% Initialise your EKF
EKF = ekf_slam();


for simulation_step = 1:400
    % The robot drives in the simulated world. The simulator works out the
    % forward kinematics of the robot and provides you with noisy velocity
    % estimates. In practice, you would get these from your inputs to the
    % robot. You can edit the simulation_world class to change the noise
    % settings.
    [u,dt] = world.drive_robot();
    % The simulator provides measurements of nearby landmarks. In practice,
    % you would use ARUCO detection for this.
    % You use the ARUCO system to measure landmark positions.
    [measurements, idx] = world.make_measurement();
    
    % PREDICTION STEP
    % Input the noisy velocities to the EKF system
%     EKF.input_velocity(dt, u, q);
    
    % UPDATE STEP
    % Input the noisy measurements to the EKF system
%     EKF.input_measurements(measurements, idx);
    
    
    % This command draws the world to help you visualise what's going on.
    % You could try and plot your EKF output on top to compare.
    world.draw();
    drawnow
end