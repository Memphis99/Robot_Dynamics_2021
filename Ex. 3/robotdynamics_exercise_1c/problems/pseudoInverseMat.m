function [ pinvA ] = pseudoInverseMat(A, lambda)
% Input: Any m-by-n matrix, and a damping factor.
% Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula

% Get the number of rows (m) and columns (n) of A
[m, n] = size(A);

if m>=n
    pinvA=((A.'*A + (lambda^2)*eye(n))^-1)*A.';
else
    pinvA=A.'*((A*A.' + (lambda^2)*eye(m))^-1);

% TODO: complete the computation of the pseudo-inverse.
% Hint: How should we account for both left and right pseudo-inverse forms?
end
