function [F_floorContact, J_floorContact, F_floorFric, J_floorFric] = computeFloorContactAndFriction_custom_ground(imc, dt, q, q0, n_nodes, n_dof)
delta = imc.delta_floor;
h = imc.h;
floor_has_friction = imc.floor_has_friction;
floor_z = imc.floor_z;
contact_stiffness = imc.k_c_floor;

K1 = 15/delta;
F_floorContact = zeros(n_dof,1);
J_floorContact = zeros(n_dof, n_dof);
F_floorFric = zeros(n_dof,1);
J_floorFric = zeros(n_dof, n_dof);
for i = 1:n_nodes
    ind = mapNodetoDOF(i);

    dist = q(ind(3)) - h - floor_z;
    n_gd = [0;0;1];

    % dist = min_dist_pt_gd(); % replace with custom functions
    % n_gd = normal_at_contact(); % replace with custom functions

    if dist > delta
        continue;
    end
    
    v = exp(-K1 * dist);
    
    f = (-2 * v * log(v + 1)) / (K1 * (v + 1)) * n_gd;
    if(isnan(f))
    assert(~isnan(f),'floor contact force is not real (NaN).');
    end

    f = f * contact_stiffness;

    J = (2*v * log(v + 1) + 2*v^2) / ((v + 1)^2) *(n_gd*transpose(n_gd));
    J = J * contact_stiffness;

    F_floorContact(ind) = F_floorContact(ind) - f; 
    J_floorContact(ind,ind) = J_floorContact(ind,ind) - J;

    if(floor_has_friction)
        curr_node = q(ind);
        pre_node = q0(ind);
        f_con = norm(f);
        % % commented 9/16 [ffr, friction_type] = computeFloorFriction_custom_gd(curr_node, pre_node, f_con, imc.mu_floor, dt, imc.velTol, n_gd);
        % % ADDED 9/16
        % % inside the per-node loop, before calling computeFloorFriction_custom_gd
        % if i == 1
        %     t_hat = q(mapNodetoDOF(2)) - q(mapNodetoDOF(1));
        % elseif i == n_nodes
        %     t_hat = q(mapNodetoDOF(n_nodes)) - q(mapNodetoDOF(n_nodes-1));
        % else
        %     t_hat = q(mapNodetoDOF(i+1)) - q(mapNodetoDOF(i-1));
        % end
        % t_hat = t_hat / norm(t_hat);
        % 
        % [ffr, friction_type] = computeFloorFriction_custom_gd(curr_node, pre_node, f_con, imc.mu_fwd, imc.mu_bwd, imc.mu_lat, dt, imc.velTol, n_gd, t_hat);
        % %
     
        % 9/20 change - commented out
        % [ffr, friction_type] = computeFloorFriction_custom_gd(curr_node, pre_node, f_con, imc.mu_floor, dt, imc.velTol, n_gd);
        % 
        % if(friction_type=="ZeroVel") 
        %     continue;
        % end
        % 
        % F_floorFric (ind) = F_floorFric (ind) + ffr;
        % 
        % Jfr = computeFloorFrictionJacobian_custom_gd(curr_node, pre_node, -f, -J, imc.mu_floor, dt, imc.velTol, friction_type, n_gd);
        % J_floorFric(ind, ind) = J_floorFric(ind, ind) + Jfr;

        % 9/20 change - added
        % friction with coefficient 1; the two real coefficients are applied by D below
        [ffr, friction_type] = computeFloorFriction_custom_gd(curr_node, pre_node, f_con, 1, dt, imc.velTol, n_gd);

        if(friction_type=="ZeroVel")
            continue;
        end

        % direction of the body at this node, in the floor plane
        ip = min(i+1, n_nodes);
        im = max(i-1, 1);
        t_hat = q(mapNodetoDOF(ip)) - q(mapNodetoDOF(im));
        t_hat = t_hat - dot(t_hat, n_gd)*n_gd;
        t_hat = t_hat / norm(t_hat);

        % D multiplies the part along the body by mu_t and the part across the body by mu_n
        D = imc.mu_n*eye(3) + (imc.mu_t - imc.mu_n)*(t_hat*t_hat');

        F_floorFric (ind) = F_floorFric (ind) + D*ffr;

        Jfr = computeFloorFrictionJacobian_custom_gd(curr_node, pre_node, -f, -J, 1, dt, imc.velTol, friction_type, n_gd);
        J_floorFric(ind, ind) = J_floorFric(ind, ind) + D*Jfr;
        % 9/20

    end

end
