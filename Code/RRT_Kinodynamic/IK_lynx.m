function [q] = IK_lynx(xyz)

L1 = 3*25.4;          %base height (in mm)
L2 = 5.75*25.4;       %shoulder to elbow length (in mm)
L3 = 7.375*25.4;      %elbow to wrist length (in mm)
PI = pi();            %Create PI constant

%Calculates the wrist center location
xw = xyz(1);
yw = xyz(2);
zw = xyz(3);

%Calculates the theta 1 needed
th1 = atan2(yw,xw);

%Parameters needed for calculating theta 2 and theta 3
r = sqrt(xw^2 + yw^2);
s = zw - L1;
D = (r^2 + s^2 - L2^2 - L3^2)/(-2*L2*L3);

%Calculates theta 3
th3 = atan2(D,sqrt(1-D^2));

%Uses theta 3 to calculate theta 2
th2 = atan2(r,s) - atan2(L3*cos(th3),L2-L3*sin(th3));

q = [th1 th2 th3];

end