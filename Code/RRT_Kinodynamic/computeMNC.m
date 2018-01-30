function [M, N, C] = computeMNC(theta, theta_dot)
%COMPUTEMNC Compute matrices for dynamic motion of the lynx robot
% M = manipulator inertia matrix
% N = gravitational force matrix
% C = coriolis matrix

theta(2) = theta(2) - pi/2;

% Initialize values
m = [1 1 1];        % Masses of the three links
l = [3 5.75 7.375]; % Lengths of three links
r = [1 1 1];        % Radii of three links
g = 9.8;            % Grav. constant

% Precompute cosines and sines to simplify eqution
c23 = cos(theta(2) + theta(3));
s23 = sin(theta(2) + theta(3));
c2 = cos(theta(2));
s2 = sin(theta(2));
c3 = cos(theta(3));
s3 = sin(theta(3));

% Moments of inertia about xyz axes of i'th link
Ix2 = (1/12)*m(2)*l(2)^2 + 0.25*m(2)*r(2)^2;
Ix3 = (1/12)*m(3)*l(3)^2 + 0.25*m(3)*r(3)^2;
Iy2 = 0.5*m(2)*r(2)^2;
Iy3 = 0.5*m(3)*r(3)^2;
Iz1 = 0.5*m(1)*r(1)^2;
Iz2 = (1/12)*m(2)*l(2)^2 + 0.25*m(2)*r(2)^2;
Iz3 = (1/12)*m(3)*l(3)^2 + 0.25*m(3)*r(3)^2;

% Compute M
M11 = Iy2*s2^2 + Iy3*s23^2 + Iz1 + Iz2*c2^2 + Iz3*c23^2 ...
    + m(2)*r(1)^2*c2^2 + m(3)*(l(1)*c2 + r(2)*c23)^2;
M22 = Ix2 + Ix3 + m(3)*l(1)^2 + m(2)*r(1)^2 + m(3)*r(2)^2 ...
    + 2*m(3)*l(1)*r(2)*c3;
M23 = Ix3 + m(3)*r(2)^2 + m(3)*l(1)*r(2)*c3;
M32 = M23;
M33 = Ix3 + m(3)*r(2)^2;

M = [M11     0       0;
    0       M22     M23;
    0       M32     M33];

% Compute N
N = [0;
    -(m(2)*g*r(1)+m(3)*g*l(1))*c2 - m(3)*r(2)*c23;
    -m(3)*g*r(2)*c23];

% Compute C
T112 = (Iy2 - Iz2 - m(2)*r(1)^2) * c2 * s2 + (Iy3 - Iz3)*c23*s23 ...
    - m(3)*(l(1)*c2 + r(2)*c23) * (l(1)*s2 + r(2)*s23);
T113 = (Iy3 - Iz3)*c23*s23 - m(3)*r(2)*s23*(l(1)*c2 + r(2)*c23);
T121 = (Iy2 - Iz2 - m(2)*r(1)^2)*c2*s2 + (Iy3 - Iz3)*c23*s23...
    - m(3)*(l(1)*c2 + r(2)*c23) * (l(1)*s2+r(2)*s23);
T131 = (Iy3 - Iz3)*c23*s23 - m(3)*r(2)*s23*(l(1)*c2 + r(2)*c23);
T211 = (Iz2 - Iy2 + m(2)*r(1)^2)*c2*s2 + (Iz3 - Iy3)*c23*s23...
    + m(3)*(l(1)*c2 + r(2)*c23)*(l(1)*s2 + r(2)*s23);
T223 = -l(1)*m(3)*r(2)*s3;
T232 = T223;
T233 = T223;
T311 = (Iz3 - Iy3)*c23*s23 + m(3)*r(2)*s23*(l(2)*c2 + r(2)*c23);
T322 = -T223;

C = [T112*theta_dot(2) + T113*theta_dot(3), T121*theta_dot(1), T131*theta_dot(1);
    T211*theta_dot(1), T223*theta_dot(3), T232*theta_dot(2) + T233*theta_dot(3);
    T311*theta_dot(1), T322*theta_dot(2), 0];

end

