function [forward_disp_final, total_disp, lateral_drift_final, ...
    yaw, mean_speed_steady, distance_per_cycle_steady, info] = ...
    custom_main_snake(amp, freq, wavelen, phase, loco_type, env_type, opts)
% Runs one snake simulation and returns locomotion metrics.
%
%   amp       : peak target bend of one joint (no unit; joint angle = 2*atan(amp/2))
%   freq      : actuation frequency (Hz)
%   wavelen   : wavelength of the body wave, in body lengths
%   phase     : constant phase offset (rad). It only shifts the time origin.
%   loco_type : 1 = land gait (uniform wave), 2 = aquatic gait (amplitude grows to the tail)
%   env_type  : 1 = land (gravity, floor contact, direction-dependent Coulomb friction)
%               2 = water (neutral buoyancy + water drag)
%   opts      : optional struct, fields (default):
%       measure_time (10)   seconds measured AFTER the start-up ramp
%       ramp_cycles  (2)    the amplitude grows smoothly from 0 over this many cycles
%       settle_time  (0)    WATER ONLY: extra seconds simulated after the ramp and BEFORE the
%                           measurement (use 60 with water_model = "stokes", which needs about
%                           60 s to reach a steady speed). Ignored on land.
%       steps_per_cycle (100) time step = min(7e-3, 1/(steps_per_cycle*freq)) (time-step check)
%       tol          (not set) if set, overrides the Newton tolerances tol, ftol, dtol (tolerance check)
%       mu_t, mu_n   (not set) if set, override the land friction coefficients (test T5)
%       water_model  ("quadratic")  "quadratic" = high-Reynolds-number drag (Taylor type)
%                                   "stokes"    = old linear RFT (low-Reynolds-number values)
%       skip_invalid (true) do not simulate gaits outside the valid range (see snake_gait_check),
%                           and return NaN if the simulated body touched itself
%       show_plots   (false) draw the snake during the run (slow)
%       verbose      (false) print Newton iterations and a summary
%       node_file    (not set) other node file, e.g. the 41-node mesh for numerics_check.m
%
% All displacement metrics use the mass-weighted centre of mass of the NODES and are
% measured over the last opts.measure_time seconds (after the ramp and the settle time).
% "Forward" is the initial body axis, from tail to head (the body is straight at t = 0).

    if nargin < 7 || isempty(opts), opts = struct(); end
    if ~isfield(opts, 'measure_time'), opts.measure_time = 10;          end
    if ~isfield(opts, 'ramp_cycles'),  opts.ramp_cycles  = 2;           end
    if ~isfield(opts, 'settle_time'),  opts.settle_time  = 0;           end
    if ~isfield(opts, 'steps_per_cycle'), opts.steps_per_cycle = 100;   end
    if env_type ~= 2, opts.settle_time = 0; end      % settling is only needed in water
    if ~isfield(opts, 'water_model'),  opts.water_model  = "quadratic"; end
    if ~isfield(opts, 'skip_invalid'), opts.skip_invalid = true;        end
    if ~isfield(opts, 'show_plots'),   opts.show_plots   = false;       end
    if ~isfield(opts, 'verbose'),      opts.verbose      = false;       end

    % "material" is also the name of a MATLAB function. Inside a function, MATLAB would call
    % that function instead of using the struct made by the script below, so declare it here.
    material = struct();
    robotDescriptionSnake   % defines nodes, edges, geom, material, sim_params
    if isfield(opts, 'node_file')            % e.g. 'experiments/snake/horizontal_rod_n41.txt' for a mesh check
        [nodes, edges, face_nodes] = inputProcessorNew(opts.node_file);
    end

    %% Check that the gait is inside the valid range of the model (before running)
    info = struct();
    info.gait = snake_gait_check(amp, wavelen, loco_type, nodes, geom.rod_r0);
    if opts.skip_invalid && ~info.gait.is_valid
        forward_disp_final = NaN; total_disp = NaN; lateral_drift_final = NaN;
        yaw = NaN; mean_speed_steady = NaN; distance_per_cycle_steady = NaN;
        info.simulated = false;
        return;
    end
    info.simulated = true;

    %% Time settings
    % At least 100 time steps in each actuation cycle (time-step error < 1 %).
    sim_params.dt = min(sim_params.dt, 1/(opts.steps_per_cycle*freq));
    if isfield(opts, 'tol')
        sim_params.tol = opts.tol; sim_params.ftol = opts.tol; sim_params.dtol = opts.tol;
    end
    T_ramp = opts.ramp_cycles/freq;                     % smooth start-up
    T_start = T_ramp + opts.settle_time;                % the measurement starts here
    sim_params.totalTime = T_start + opts.measure_time;
    sim_params.verbose = opts.verbose;

    %% Environment
    switch env_type
        case 1  % Land: gravity, floor contact and direction-dependent Coulomb friction
            env.ext_force_list = ["gravity", "floorContact", "floorFriction"];
            env.g = [0, 0, -9.81]';
            env.contact_stiffness = 1000;
            env.mu = 0.25;      % not used by the snake friction law below (kept for other code)
            env.mu_t = 0.11;    % friction along the body  (Hu et al. 2009, forward sliding)
            env.mu_n = 0.20;    % friction across the body (Hu et al. 2009, sideways sliding)
            if isfield(opts, 'mu_t'), env.mu_t = opts.mu_t; end
            if isfield(opts, 'mu_n'), env.mu_n = opts.mu_n; end
            env.velTol = 1e-2;
            % The snake starts AT REST on the floor (contact force = weight), so it does
            % not jump into the air at t = 0.
            env.floor_z = floor_height_at_rest(nodes, geom, material, env);

        case 2  % Water (density 1000 = rod density, so the snake is neutrally buoyant)
            env.g = [0, 0, -9.81]';
            env.rho = 1000;
            if strcmp(opts.water_model, "quadratic")
                % High Reynolds number (Re ~ 1e3 to 1e4): drag proportional to speed^2
                % (resistive model of Taylor 1952). See get_hydro_drag_force.m.
                env.ext_force_list = ["gravity", "buoyancy", "hydroDrag"];
                env.Cn = 1.0;    % drag coefficient across the body (circular cylinder, Re_d ~ 1e3)
                env.Ct = 0.02;   % skin-friction coefficient along the body (Re_L ~ 1e4)
            elseif strcmp(opts.water_model, "stokes")
                % Old model: linear RFT with slender-body Stokes-flow coefficients for water.
                % Valid only for Re << 1; with these values the snake needs ~60 s to reach
                % a steady speed (see the plan, problem P5).
                env.ext_force_list = ["gravity", "buoyancy", "rft"];
                env.ct = 1.53e-3;
                env.cn = 2.46e-3;
            else
                error('Unknown opts.water_model');
            end
        otherwise
            error('Invalid env_type');
    end

    % set values for parameters
    amplitude = amp;
    frequency = freq;
    spatial_wavelength = wavelen;
    phase_offset = phase;

    % create geometry
    [nodes, edges, rod_edges, shell_edges, rod_shell_joint_edges, rod_shell_joint_total_edges, face_nodes, face_edges, face_shell_edges, ...
        elStretchRod, elStretchShell, elBendRod, elBendSign, elBendShell, sign_faces, face_unit_norms]...
        = createGeometry(nodes, edges, face_nodes);

    % intialize twist angles for rod-edges to 0
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

    % shell bending spring (not used by the snake)
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
    softRobot = computeSpaceParallel(softRobot);
    theta = softRobot.q0(3*softRobot.n_nodes+1:3*softRobot.n_nodes+softRobot.n_edges_dof);
    [softRobot.m1, softRobot.m2] = computeMaterialDirectors(softRobot.a1,softRobot.a2,theta);
    bend_twist_springs = setkappa(softRobot, bend_twist_springs);
    softRobot.undef_refTwist = computeRefTwist_bend_twist_spring ...
        (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
        zeros(n_bend_twist,1));
    softRobot.refTwist = computeRefTwist_bend_twist_spring ...
        (bend_twist_springs, softRobot.a1, softRobot.tangent, ...
        softRobot.undef_refTwist);

    %% Boundary Conditions (none: the snake is free)
    softRobot.fixed_nodes = fixed_node_indices;
    for i=1:size(softRobot.Edges,1)
        if ( ismember(softRobot.Edges(i,1),fixed_node_indices) && ismember(softRobot.Edges(i,2),fixed_node_indices) )
            fixed_edge_indices = [fixed_edge_indices, i];
        end
    end
    if(sim_params.TwoDsim)
        fixed_edge_indices = [fixed_edge_indices, 1:softRobot.n_edges_dof];
    end
    softRobot.fixed_edges = fixed_edge_indices;
    [softRobot.fixedDOF, softRobot.freeDOF] = FindFixedFreeDOF(softRobot.fixed_nodes, softRobot.fixed_edges, softRobot.n_DOF, softRobot.n_nodes);

    if opts.show_plots
        plot_MultiRod(softRobot, 0.0, sim_params,environment,imc);
    end

    %% Time stepping
    Nsteps = round(sim_params.totalTime/sim_params.dt);
    n_nodes = softRobot.n_nodes;
    t_log = (0:Nsteps)*sim_params.dt;           % column k of the logs is the state at t_log(k)
    Xn = zeros(n_nodes, Nsteps+1); Yn = Xn; Zn = Xn;
    Xn(:,1) = softRobot.q0(1:3:3*n_nodes);      % only the node DOFs (the last DOFs are twist angles)
    Yn(:,1) = softRobot.q0(2:3:3*n_nodes);
    Zn(:,1) = softRobot.q0(3:3:3*n_nodes);
    current_kappa_bar = zeros(n_bend_twist,2);
    ctime = 0; % current time
    tic;

    for timeStep = 1:Nsteps
        if(sim_params.use_midedge)
            tau_0 = updatePreComp_without_sign(softRobot.q, softRobot);
        else
            tau_0 = [];
        end

        %% actuate (smooth start: the amplitude grows from 0 to amp during the ramp)
        if ctime < T_ramp
            ramp = 0.5*(1 - cos(pi*ctime/T_ramp));
        else
            ramp = 1;
        end
        current_kappa_bar(:,2) = actuate_snake(n_bend_twist, ctime, ramp*amplitude, frequency, spatial_wavelength, phase_offset, loco_type)';
        bend_twist_springs = actuatekappa(bend_twist_springs, current_kappa_bar);

        %% Implicit time step
        [softRobot, stretch_springs, bend_twist_springs, hinge_springs] = ...
            timeStepper(softRobot, stretch_springs, bend_twist_springs, hinge_springs, triangle_springs, tau_0,environment,imc, sim_params);

        ctime = ctime + sim_params.dt;
        softRobot.q0 = softRobot.q;

        %% Logging (node positions only)
        Xn(:,timeStep+1) = softRobot.q(1:3:3*n_nodes);
        Yn(:,timeStep+1) = softRobot.q(2:3:3*n_nodes);
        Zn(:,timeStep+1) = softRobot.q(3:3:3*n_nodes);

        if opts.show_plots && mod(timeStep, sim_params.plotStep) == 0
            plot_MultiRod(softRobot, ctime, sim_params, environment, imc);
        end
    end
    info.runtime_s = toc;

    %% Metrics
    m_node = softRobot.massVec(1:3:3*n_nodes);          % node masses (end nodes have half mass)
    com_x = (m_node' * Xn) / sum(m_node);
    com_y = (m_node' * Yn) / sum(m_node);

    head_node = 1;
    tail_node = n_nodes;
    e_fwd = [Xn(head_node,1) - Xn(tail_node,1), Yn(head_node,1) - Yn(tail_node,1)];
    e_fwd = e_fwd / norm(e_fwd);                         % initial body axis, tail -> head
    e_lat = [-e_fwd(2), e_fwd(1)];

    k0 = find(t_log >= T_start - 1e-9, 1, 'first');      % start of the measurement (end of ramp + settle time)
    dx = com_x - com_x(k0);
    dy = com_y - com_y(k0);
    forward_disp = dx*e_fwd(1) + dy*e_fwd(2);
    lateral_disp = dx*e_lat(1) + dy*e_lat(2);

    forward_disp_final  = forward_disp(end);             % signed, positive = toward the head
    lateral_drift_final = lateral_disp(end);
    total_disp          = hypot(dx(end), dy(end));

    % yaw: mean body heading (head - tail direction) over the last cycle, minus the initial heading
    heading = unwrap(atan2(Yn(head_node,:) - Yn(tail_node,:), Xn(head_node,:) - Xn(tail_node,:)));
    n_cyc = min(numel(heading), max(1, round(1/(frequency*sim_params.dt))));
    yaw = mean(heading(end-n_cyc+1:end)) - heading(1);

    % steady values: second half of the measurement window
    k_half = find(t_log >= t_log(k0) + 0.5*opts.measure_time - 1e-9, 1, 'first');
    mean_speed_steady = (forward_disp(end) - forward_disp(k_half)) / (t_log(end) - t_log(k_half));
    distance_per_cycle_steady = mean_speed_steady / frequency;

    % direction of travel relative to the initial body axis
    info.travel_angle_deg = atan2(lateral_disp(end), forward_disp(end))*180/pi;

    % self-contact check on the SIMULATED body (every 10 steps)
    info.min_gap = inf;
    for k = 1:10:Nsteps+1
        info.min_gap = min(info.min_gap, body_min_gap([Xn(:,k), Yn(:,k), Zn(:,k)], 3));
    end
    info.self_contact = info.min_gap < 2*geom.rod_r0;
    % A run in which the SIMULATED body came closer to itself than one body diameter is
    % not physical (the model has no self-contact force). Report it as NaN.
    if opts.skip_invalid && info.self_contact
        forward_disp_final = NaN; total_disp = NaN; lateral_drift_final = NaN;
        yaw = NaN; mean_speed_steady = NaN; distance_per_cycle_steady = NaN;
    end
    info.max_height = max(Zn(:)) - Zn(1,1);              % should stay ~0 on land (no jump)
    info.t = t_log; info.forward_disp_t = forward_disp; info.lateral_disp_t = lateral_disp;
    info.T_ramp = T_ramp; info.T_start = T_start; info.dt = sim_params.dt;

    if opts.verbose
        fprintf('\n===== Locomotion Metrics Summary =====\n');
        fprintf('amp %.4f  freq %.4f  wavelength %.4f  loco %d  env %d\n', amp, freq, wavelen, loco_type, env_type);
        fprintf('Forward displacement (%.1f s):  %.6f m\n', opts.measure_time, forward_disp_final);
        fprintf('Lateral drift:                 %.6f m\n', lateral_drift_final);
        fprintf('Travel direction:              %.2f deg\n', info.travel_angle_deg);
        fprintf('Net yaw:                       %.4f rad (%.2f deg)\n', yaw, yaw*180/pi);
        fprintf('Mean speed (steady):           %.6f m/s\n', mean_speed_steady);
        fprintf('Distance per cycle (steady):   %.6f m/cycle\n', distance_per_cycle_steady);
        fprintf('Smallest body gap:             %.2f mm (2r = %.2f mm)\n', 1e3*info.min_gap, 2e3*geom.rod_r0);
        fprintf('Run time:                      %.1f s\n', info.runtime_s);
        fprintf('========================================\n\n');
    end
end


function floor_z = floor_height_at_rest(nodes, geom, material, env)
% Floor height such that a node at z = 0 is exactly at rest on the floor
% (contact force = weight of one node). Uses the same contact law as
% computeFloorContactAndFriction_custom_ground.m (delta = rod radius).
l_seg  = norm(nodes(2,:) - nodes(1,:));
m_node = material.density*pi*geom.rod_r0^2*l_seg;   % mass of one interior node
w      = m_node*abs(env.g(3));                      % its weight
K1     = 15/geom.rod_r0;
Fc     = @(d) env.contact_stiffness*2*exp(-K1*d).*log(1+exp(-K1*d))./(K1*(1+exp(-K1*d)));
d_eq   = fzero(@(d) Fc(d) - w, [0, geom.rod_r0]);    % gap between rod surface and floor at rest
floor_z = -(geom.rod_r0 + d_eq);
end
