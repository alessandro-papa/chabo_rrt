function U = local_motion_planning(x_initial, x_goal, Pinv_cell, D_cell, Nmax, u_min, u_max, n_increment)
    % Default increment value
    if(nargin < 8)
        n_increment = 1;
    end
    % Sizes
    nbr_states = length(x_initial);
    nbr_inputs = length(u_min);
    % Success flag
    success_flag = false;
    for n = nbr_states:n_increment:Nmax
        % Connection
        Pn_pinv = Pinv_cell{n,1};
        Dn      = D_cell{n,1};
        u = Pn_pinv*(x_goal-Dn*x_initial);
        % Check whether input is valid
        input_valid = true;
        for i = 1:n
            un = u(1+(i-1)*nbr_inputs:i*nbr_inputs,1);
            if(sum(un>u_min & un < u_max) < nbr_inputs)
                input_valid = false;
                break;
            end
        end
        if(input_valid)
            success_flag = true;
            break;
        end
    end
    % Check for success
    if(~success_flag)
        U = [];
    else
        U = reshape(u, nbr_inputs, n);
    end
end

