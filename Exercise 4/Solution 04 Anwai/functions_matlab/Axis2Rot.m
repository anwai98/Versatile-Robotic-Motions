function [R] = Axis2Rot(AxisAngle)
%AXIS2ROT Summary of this function goes here
%   Detailed explanation goes here
rx = AxisAngle(1);
ry = AxisAngle(2);
rz = AxisAngle(3);
theta = AxisAngle(4);
R = [rx*rx*(1-cos(theta))+cos(theta) rx*ry*(1-cos(theta))-rz*sin(theta) rx*rz*(1-cos(theta))+ry*sin(theta);
     rx*ry*(1-cos(theta))+rz*sin(theta) ry*ry*(1-cos(theta))+cos(theta) ry*rz*(1-cos(theta))-rx*sin(theta);
     rx*rz*(1-cos(theta))-ry*sin(theta) ry*rz*(1-cos(theta))+rx*sin(theta) rz*rz*(1-cos(theta))+cos(theta)];
end

