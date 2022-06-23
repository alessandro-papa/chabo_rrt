function [Pinv_cell, D_cell] = precompute_Pinv_D(A, B, N)
    % Sizes
    nbr_states = size(A,1);
    nbr_inputs = size(B,2);
    % Allocate
    Pinv_cell = cell(N, 1);
    D_cell    = cell(N, 1);
    % Construct P
    Pn = B;
    Pc = cell(N+1,1);
    Pc{1,1} = zeros(5, 2);
    for n = 1:N
        Pc{n+1,1} = Pn;
        Pn = A*Pn;
    end
    select_ind = tril(toeplitz(1:N))+1;
    P = cell2mat(Pc(select_ind));
    % Compute the elements of the Pinv cell
    for n = 1:N
        Pn = P(1+(n-1)*nbr_states:n*nbr_states, 1:n*nbr_inputs);
        Pinv_cell{n,1} = pinv(Pn);
    end
    % Construct D
    Dn = A;
    for n = 1:N
        D_cell{n,1} = Dn;
        Dn = A*Dn;
    end
end

