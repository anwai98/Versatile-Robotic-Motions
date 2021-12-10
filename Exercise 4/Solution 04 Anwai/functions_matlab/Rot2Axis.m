function [AxisAngle] = Rot2Axis(R)
%ROT2AXIS Summary of this function goes here
%   Detailed explanation goes here

AxisAngle = zeros(4,1);

angle = acos((R(1,1) + R(2,2) + R(3,3) - 1)/2);

var = (1/(2*sin(angle)));
r = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)] * var;

AxisAngle = [r; angle];

end