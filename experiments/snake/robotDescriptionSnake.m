% input: robotDescriptionSnake.m  (called by custom_main_snake.m)

sim_params.static_sim = 0;
sim_params.TwoDsim = 0;
sim_params.use_midedge = false;
sim_params.use_lineSearch = false;
sim_params.showFrames = false;
sim_params.logStep = 1;
sim_params.log_data = false;      % custom_main_snake.m logs the node positions itself
sim_params.bergou_DER = 0;
sim_params.FDM = 0;

% Largest time step (s). custom_main_snake.m makes it smaller if needed, so that
% each actuation cycle has at least 100 steps: dt = min(7e-3, 1/(100*freq)).
sim_params.dt = 7e-3;

% Maximum number of iterations in Newton Solver
sim_params.maximum_iter = 50;

% Total simulation time: set in custom_main_snake.m (ramp + measurement time)
sim_params.totalTime = 10;

% How often to plot when opts.show_plots = true
sim_params.plotStep = 10;

%% Input text file
inputFileName = 'experiments/snake/horizontal_rod_n21.txt';   % 21 nodes, body length 0.15 m
[nodes, edges, face_nodes] = inputProcessorNew(inputFileName);

%% Geometry and material
geom.shell_h = 0;
geom.rod_r0 = 0.003;              % rod radius (m) -- report this value in the paper

material.density = 1000;          % kg/m^3 (equal to water: neutrally buoyant)
material.youngs_rod = 2e6;
material.youngs_shell = 0;
material.poisson_rod = 0.5;
material.poisson_shell = 0;

%% Newton solver tolerances (checked: results do not change with 1e-7)
sim_params.tol = 1e-4;
sim_params.ftol = 1e-4;
sim_params.dtol = 1e-4;           % must stay 1e-4 (1e-2 stops before friction is applied)

%% Boundary conditions
fixed_node_indices = [];
fixed_edge_indices = [];

%% logging
input_log_node = size(nodes,1);

%% Plot dimensions (only used when opts.show_plots = true)
sim_params.plot_x = [-1.0, 0.2];
sim_params.plot_y = [-0.1, 0.1];
sim_params.plot_z = [-0.05, 0.05];
sim_params.view = "xy";
