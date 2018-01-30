function [I] = q_indices(q)
% Returns closest indices to a configuration q

av = linspace(-1.4, 1.4, n_step);
bv = linspace(-1.2, 1.4, n_step);
cv = linspace(-1.8, 1.7, n_step);
dv = linspace(-1.9, 1.7, n_step);

I = zeros(5, 1); % return column vector of indices

% Find closest points
[~, I(1)] = min(abs(av - q(1)));
[~, I(2)] = min(abs(bv - q(2)));
[~, I(3)] = min(abs(bv - q(3)));
[~, I(4)] = min(abs(bv - q(4)));

end

