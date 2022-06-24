function d = chabo_state_distance(x, Y)
    % Allocate return structure
    N = size(Y,1);
    d = zeros(N,1);
    % Scaling 
    ygain = 1./[pi; 8; pi; 4; 420];     % distance in state space
    % Scale x
    x = x';
    x = ygain.*x;
    for n = 1:N
        y = Y(n, :)';
        y = ygain.*y;
        d(n,1) = norm(x-y);
    end
end

 