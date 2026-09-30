function [forward_disp_final, total_disp, lateral_drift_final, ...
    yaw, mean_speed_steady, distance_per_cycle_steady] = ...
    custom_main_snake(amp, freq, wavelen, phase, loco_type, env_type)
%function dist = custom_main_snake(amp, freq, wavelen, phase, loco_type, env_type)

    % % Examples:
    robotDescriptionSnake

    switch env_type

        case 1  % Land environment (anisotropic friction)
            % REAL LAND ENVIRONMENT
            env.ext_force_list = ["gravity", "floorContact", "floorFriction"];
            % environment parameters
            env.g = [0, 0, -9.81]';
            env.contact_stiffness = 1000;
            env.mu = 0.25;
            % ADDED 9/16
            env.mu_fwd = 0.49;
            env.mu_bwd = 0.79;
            env.mu_lat = 1.01;

            % 9/20 change
            env.mu_t = 0.11;
            env.mu_n = 0.20;
            %

            % 9/20 changed from -0.001
            env.floor_z = -geom.rod_r0;
            env.velTol = 1e-2;

            % env.ct = 0.01;
            % env.cn = 0.1;
    
        case 2  % Aquatic environment (more isotropic drag)
            % AQUATIC ENVIRONMENT
            env.ext_force_list = ["gravity", "buoyancy", "rft"];  
            env.g = [0, 0, -9.81]';
            env.rho = 1000;
            env.Cd = 0.3;

            % rft
            env.ct = 1.53e-3;
            env.cn = 2.46e-3;
            env.floor_z = -0.001;

            % env.ct = 0.03;
            % env.cn = 0.06;
    
    end

    env.floor_z = -0.001;

    material.density

    % set values for parameters
    amplitude = amp;
    frequency = freq;             
    spatial_wavelength = wavelen;
    phase_offset = phase;

    % run simulation here

    % % add to path
    % addpath springs/
    % addpath util_functions/
    % addpath contact_functions/
    % addpath rod_dynamics/
    % addpath shell_dynamics/
    % addpath external_forces/
    % addpath adaptive_stepping/
    % addpath logging/
    % addpath(genpath('experiments'));



    % create geometry
    [nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, face_shell_edges, ...
        elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
        = createGeometry(nodes, edges, face_nodes);

    % intialize twist angles for rod-edges to 0: this should be changed if one
    % wants to start with a non-zero intial twist
    twist_angles=zeros(size(rod_edges,1)+size(rod_shell_joint_total_edges,1),1);

    % create environment and imc structs
    [environment,imc] = createEnvironmentAndIMCStructs(env,geom,material,sim_params);


    %% Create the soft robot structure
    softRobot = MultiRod(geom, material, twist_angles,...
        nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, ...
        face_nodes, sign_faces, face_edges, face_shell_edges, sim_params, environment);

    %% Creating stretching, bending, twisting and hinge springs

    n_stretch = size(elStretchRod,1) + size(elStretchShell,1);
    n_bend_twist = size(elBendRod,1);

    % stretching spring
    if(n_stretch==0)
        stretch_springs = [];
    else
        for s=1:n_stretch
            if (s <= size(elStretchRod,1)) % rod
                stretch_springs (s) = stretchSpring (...
                    softRobot.refLen(s), elStretchRod(s,:),softRobot);

            else % shell
                stretch_springs (s) = stretchSpring (...
                    softRobot.refLen(s), ...
                    elStretchShell(s-(size(elStretchRod,1)),:), ...
                    softRobot, softRobot.ks(s));
            end
        end
    end

    % bending and twisting spring
    if(n_bend_twist==0)
        bend_twist_springs = [];
    else
        for b=1:n_bend_twist
            bend_twist_springs(b) = bendTwistSpring ( ...
                elBendRod(b,:), elBendSign(b,:), [0 0], 0, softRobot);
        end
    end

    % shell bending spring
    n_hinge = size(elBendShell,1);
    n_triangle = softRobot.n_faces;
    if(n_triangle==0)
        hinge_springs = [];
        triangle_springs = [];
    else
        if(~sim_params.use_midedge)
            triangle_springs = [];
            for h=1:n_hinge
                hinge_springs(h) = hingeSpring (...
                    0, elBendShell(h,:), softRobot, softRobot.kb);
            end
            hinge_springs = setThetaBar(hinge_springs, softRobot);
        else
            hinge_springs = [];
            for t=1:n_triangle
                triangle_springs(t) = triangleSpring(softRobot.face_nodes_shell(t,:), softRobot.face_edges(t,:), softRobot.face_shell_edges(t,:), softRobot.sign_faces(t,:), softRobot);
            end
        end
    end

    %% Prepare system
    % Reference frame (Space parallel transport at t=0)
    softRobot = computeSpaceParallel(softRobot);

    % Material frame from reference frame and twist angle
    theta = softRobot.q0(3*softRobot.n_nodes+1:3*softRobot.n_nodes+softRobot.n_edges_dof); % twist angle
    [softRobot.m1, softRobot.m2] = computeMaterialDirectors(softRobot.a1,softRobot.a2,theta);

    % Set rod natural curvature
    bend_twist_springs = setkappa(softRobot, bend_twist_springs);

    % Reference twist
    softRobot.undef_refTwist = computeRefTwist_bend_twist_spring ...
        (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
        zeros(n_bend_twist,1));
    softRobot.refTwist = computeRefTwist_bend_twist_spring ...
        (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
        softRobot.undef_refTwist);

    %% Boundary Conditions

    softRobot.fixed_nodes = fixed_node_indices;
    for i=1:size(softRobot.Edges,1)
        if ( ismember(softRobot.Edges(i,1),fixed_node_indices) && ismember(softRobot.Edges(i,2),fixed_node_indices) )
            fixed_edge_indices = [fixed_edge_indices, i];
        end
    end
    if(sim_params.TwoDsim)
        fixed_edge_indices = [fixed_edge_indices, 1:softRobot.n_edges_dof]; % all rod thetas are fixed if it is a 2D sim
    end
    softRobot.fixed_edges = fixed_edge_indices;
    [softRobot.fixedDOF, softRobot.freeDOF] = FindFixedFreeDOF(softRobot.fixed_nodes, softRobot.fixed_edges, softRobot.n_DOF, softRobot.n_nodes);

    % Visualize initial configuration and the fixed and free nodes: free nodes - blue, fixed - red
    % commented out 9/27 - plot
    plot_MultiRod(softRobot, 0.0, sim_params,environment,imc);

    %% Initial conditions on velocity / angular velocity (if any)


    %% Time stepping scheme

    Nsteps = round(sim_params.totalTime/sim_params.dt);
    ctime = 0; % current time
    time_arr = linspace(0,sim_params.totalTime,Nsteps);

    % containers for logging data
    current_pos_x = zeros(Nsteps,1);
    current_pos_y = zeros(Nsteps,1);
    current_pos_z = zeros(Nsteps,1);
    log_node = input_log_node;
    current_pos_x(1) = softRobot.q0(3*log_node-2);
    current_pos_y(1) = softRobot.q0(3*log_node-1);
    current_pos_z(1) = softRobot.q0(3*log_node);

    % ADDED for COM
    com_pos_x = zeros(Nsteps,1);
    com_pos_y = zeros(Nsteps,1);
    com_pos_z = zeros(Nsteps,1);

    x_nodes = softRobot.q0(1:3:end);
    y_nodes = softRobot.q0(2:3:end);
    z_nodes = softRobot.q0(3:3:end);

    com_pos_x(1) = mean(x_nodes);
    com_pos_y(1) = mean(y_nodes);
    com_pos_z(1) = mean(z_nodes);

    % ADDED to measure 9/7
    head_node = 1;
    tail_node = softRobot.n_nodes;

    head_pos_x = zeros(Nsteps,1); head_pos_y = zeros(Nsteps,1);
    tail_pos_x = zeros(Nsteps,1); tail_pos_y = zeros(Nsteps,1);

    head_pos_x(1) = softRobot.q0(3*head_node-2);
    head_pos_y(1) = softRobot.q0(3*head_node-1);
    tail_pos_x(1) = softRobot.q0(3*tail_node-2);
    tail_pos_y(1) = softRobot.q0(3*tail_node-1);
    %

    dof_with_time = zeros(softRobot.n_DOF+1,Nsteps);
    dof_with_time(1,:) = time_arr;
    current_kappa_bar = zeros(n_bend_twist,2);

    L = 0;
    for e = 1:softRobot.n_edges
        % get node indices for edge e (edge e connects nodes n1 and n2)
        n1 = softRobot.Edges(e,1);
        n2 = softRobot.Edges(e,2);

        % get node positions
        x1 = softRobot.q0(mapNodetoDOF(n1));
        x2 = softRobot.q0(mapNodetoDOF(n2));

        % find Euclidean distance between nodes to get edge length
        % add to total length
        L = L + norm(x2 - x1);
    end


    for timeStep = 1:Nsteps
        if(sim_params.static_sim)
            environment.g = timeStep*environment.static_g/Nsteps; % ramp gravity
        end
        %% Precomputation at each timeStep: midedge normal shell bending
        if(sim_params.use_midedge)
            tau_0 = updatePreComp_without_sign(softRobot.q, softRobot);
        else
            tau_0 = [];
        end

        %% actuate
        current_kappa_bar(:,2) = actuate_snake(n_bend_twist, ctime, amplitude, frequency, spatial_wavelength, phase_offset, loco_type)';
        bend_twist_springs = actuatekappa(bend_twist_springs, current_kappa_bar);

        %%  Implicit stepping error iteration
        [softRobot, stretch_springs, bend_twist_springs, hinge_springs] = ...
            timeStepper(softRobot, stretch_springs, bend_twist_springs, hinge_springs, triangle_springs, tau_0,environment,imc, sim_params);

        ctime = ctime + sim_params.dt;


        % %% --- ADDED: ACTUATION ENERGY LOGGING (ADD THIS BLOCK) ---
        % for b = 1:n_bend_twist
        % 
        %     % bending + twisting effort
        %     tau_b = norm(bend_twist_springs(b).dFb) + norm(bend_twist_springs(b).dFt);
        % 
        %     % curvature rate
        %     theta_dot_b = (bend_twist_springs(b).kappaBar - kappaBar_prev(b,:)) ...
        %         / sim_params.dt;
        % 
        %     % power = sum over the 2 bending directions
        %     P_b(b, timeStep) = abs(tau_b * norm(theta_dot_b));
        % 
        %     % store previous curvature
        %     kappaBar_prev(b,:) = bend_twist_springs(b).kappaBar;
        % end

        % P_total(timeStep) = sum(P_b(:,timeStep));


        % Update q
        softRobot.q0 = softRobot.q;

        %% Logging and animation
        current_pos_x(timeStep) = softRobot.q0(3*log_node-2);
        current_pos_y(timeStep) = softRobot.q0(3*log_node-1);
        current_pos_z(timeStep) = softRobot.q0(3*log_node);

        % ADDED for COM
        x_nodes = softRobot.q(1:3:end);
        y_nodes = softRobot.q(2:3:end);
        z_nodes = softRobot.q(3:3:end);
        
        com_pos_x(timeStep) = mean(x_nodes);
        com_pos_y(timeStep) = mean(y_nodes);
        com_pos_z(timeStep) = mean(z_nodes);

        % ADDED for measure 9/7
        head_pos_x(timeStep) = softRobot.q(3*head_node-2);
        head_pos_y(timeStep) = softRobot.q(3*head_node-1);
        tail_pos_x(timeStep) = softRobot.q(3*tail_node-2);
        tail_pos_y(timeStep) = softRobot.q(3*tail_node-1);
        %

        if(sim_params.log_data)
            if mod(timeStep, sim_params.logStep) == 0
                dof_with_time(2:end,timeStep) =  softRobot.q;
            end
        end
        % commented out 9/27 - plot
        if mod(timeStep, sim_params.plotStep) == 0
            plot_MultiRod(softRobot, ctime, sim_params, environment, imc);

            if abs(ctime-0.2) < sim_params.dt
                savefig(gcf,'snake_t1.fig')
            elseif abs(ctime-0.4) < sim_params.dt
                savefig(gcf,'snake_t2.fig')
            elseif abs(ctime-0.6) < sim_params.dt
                savefig(gcf,'snake_t3.fig')
            elseif abs(ctime-0.8) < sim_params.dt
                savefig(gcf,'snake_t4.fig')
            end
        end
    end

    % % ADDED
    % E_act = trapz(time_arr, P_total);
    % fprintf('Total actuation energy: %.6f J\n', E_act);
    % 
    % disp(softRobot.refLen)

    % ADDED for measure 9/7
    % establish reference heading (normalized vectors)
    init_heading = [head_pos_x(1)-tail_pos_x(1), head_pos_y(1)-tail_pos_y(1)];
    init_heading = init_heading / norm(init_heading);
    lateral_dir  = [-init_heading(2), init_heading(1)];

    
    disp_x = com_pos_x - com_pos_x(1);
    disp_y = com_pos_y - com_pos_y(1);

    % how far forward it has traveled
    forward_disp = disp_x*init_heading(1) + disp_y*init_heading(2);
    % how far sideways it has drifted
    lateral_disp = disp_x*lateral_dir(1)  + disp_y*lateral_dir(2);


    % Final signed forward/backward displacement
    forward_disp_final = forward_disp(end);

    % Total horizontal displacement of COM
    total_disp = sqrt( ...
        (com_pos_x(end) - com_pos_x(1))^2 + ...
        (com_pos_y(end) - com_pos_y(1))^2 );


    % lateral drift metrics
    lateral_drift_final = lateral_disp(end);

    % yaw
    yaw_angle = atan2(head_pos_y - tail_pos_y, head_pos_x - tail_pos_x);
    yaw_angle = unwrap(yaw_angle);
    net_yaw   = yaw_angle(end) - yaw_angle(1);
    % net yaw in radians
    yaw = net_yaw;

    % mean speed after startup transient
    period = 1/frequency;
    n_transient_cycles = 2;
    t_transient = min(n_transient_cycles * period, 0.5*sim_params.totalTime);  % never eat more than half the run

    idx0 = find(time_arr >= t_transient, 1, 'first');
    if isempty(idx0), idx0 = 1; end

   
    steady_time   = time_arr(end) - time_arr(idx0);
    steady_disp   = forward_disp(end) - forward_disp(idx0);
    mean_speed_steady = steady_disp / steady_time;

    % distance per actuation cycle
    total_cycles = sim_params.totalTime * frequency;
    distance_per_cycle_overall = forward_disp(end) / total_cycles;

    steady_cycles = (time_arr(end) - t_transient) * frequency;
    if steady_cycles < 0.5
        warning('Steady-state window contains less than half a cycle; distance-per-cycle (steady) may be unreliable.');
    end
    distance_per_cycle_steady  = steady_disp / steady_cycles;
    %


    %% Saving data


    [rod_data,shell_data] = logDataForRendering(dof_with_time, softRobot, Nsteps, sim_params.static_sim);

    filename = "node_trajectory.xls";
    writematrix(time_arr', filename, Sheet=1,Range='A1');
    writematrix(current_pos_x, filename, Sheet=1,Range='B1');
    writematrix(current_pos_y, filename, Sheet=1,Range='C1');
    writematrix(current_pos_z, filename, Sheet=1,Range='D1');

    %% ADDED 9/7 --- Print summary of movement metrics ---
    fprintf('\n===== Locomotion Metrics Summary =====\n');
    fprintf('Lateral drift (final):        %.6f m\n', lateral_drift_final);
    fprintf('Net yaw:                      %.4f rad (%.2f deg)\n', net_yaw, rad2deg(net_yaw));
    fprintf('Mean speed (steady-state):    %.6f m/s\n', mean_speed_steady);
    fprintf('Distance per cycle (overall): %.6f m/cycle\n', distance_per_cycle_overall);
    fprintf('Distance per cycle (steady):  %.6f m/cycle\n', distance_per_cycle_steady);

    fprintf('amp: %.6f\n', amp);
    fprintf('freq: %.6f\n', freq);
    fprintf('wavelength: %.6f\n', wavelen);
    


    fprintf('========================================\n\n');

    %% Plots
    % % time trajectory
    % figure()
    % plot(time_arr,current_pos_x, time_arr, current_pos_y, time_arr, current_pos_z);
    % title('time trajectory of the node')
    % legend(['x'; 'y'; 'z'])
    % xlabel('t [s]')
    % ylabel('z [m]')
    % 
    % % space trajectory
    % figure()
    % plot(time_arr, beg_current_pos_y, 'g', 'LineWidth', 2);
    % hold on;
    % plot(time_arr, mid_current_pos_y, 'b', 'LineWidth', 2);
    % plot(time_arr, current_pos_y, 'r', 'LineWidth', 2);
    % 
    % title('space trajectory of the node')
    % legend(['x'; 'y'; 'z'])
    % xlabel('x [m]')
    % ylabel('y [m]')
    % zlabel('z [m]')


    % ORIGINAL
    % % space trajectory
    % figure()
    % plot3(current_pos_x, current_pos_y, current_pos_z);
    % title('space trajectory of the node')
    % legend(['x'; 'y'; 'z'])
    % xlabel('x [m]')
    % ylabel('y [m]')
    % zlabel('z [m]')

    % compute distance
    %dist = abs(current_pos_x(end) - current_pos_x(1));

    % % Plot total displacement
    % dist = sqrt( ...
    %      (com_pos_x(end) - com_pos_x(1))^2 + ...
    %      (com_pos_y(end) - com_pos_y(1))^2 );

end
