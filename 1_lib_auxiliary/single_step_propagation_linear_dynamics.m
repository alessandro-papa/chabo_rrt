function x_next = single_step_propagation_linear_dynamics(x, u, A, B)
    x_next = A*x + B*u;
end

