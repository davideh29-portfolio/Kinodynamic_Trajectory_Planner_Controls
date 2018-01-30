function [X, T] = updateQ(q)

T = zeros(4,4,3);

%Important parameters for the robot
L1 = 3*25.4;          %base height (in mm)
L2 = 5.75*25.4;       %shoulder to elbow length (in mm)
L3 = 7.375*25.4;      %elbow to wrist length (in mm)
PI = pi();            %PI constant


%Frame 1 w.r.t Frame 0
A1 = [cos(q(1)) -sin(q(1))*cos(-PI/2)  sin(q(1))*sin(-PI/2)  0;
      sin(q(1))  cos(q(1))*cos(-PI/2) -cos(q(1))*sin(-PI/2)  0;
              0            sin(-PI/2)            cos(-PI/2) L1;
              0                     0                  0     1];
          
%Frame 2 w.r.t Frame 1          
A2 = [cos(q(2)-(PI/2)) -sin(q(2)-(PI/2))  0   L2*cos(q(2)-(PI/2));
      sin(q(2)-(PI/2))  cos(q(2)-(PI/2))  0   L2*sin(q(2)-(PI/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 3 w.r.t Frame 2
A3 = [cos(q(3)+(PI/2)) -sin(q(3)+(PI/2))  0   L3*cos(q(3)+(PI/2));
      sin(q(3)+(PI/2))  cos(q(3)+(PI/2))  0   L3*sin(q(3)+(PI/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Puts the Homogeneous Tranformations in T Matrix
T(:,:,1) = A1;
T(:,:,2) = A2;
T(:,:,3) = A3;

%Position of First Joint (Base Revolute)
X(1,:) = [0 0 0 1];
%Position of Second Joint (Shoulder Revolute)
X(2,:) = ((A1)*[0;0;0;1])';
%Position of Third Joint (Elbow Revolute)
X(3,:) = ((A1*A2)*[0;0;0;1])';
%Position of Fourth Joint (End effector)
X(4,:) = ((A1*A2*A3)*[0;0;0;1])';

%Outputs the 4x3 of the locations of each joint in the Base Frame
X = X(:,1:3);

end