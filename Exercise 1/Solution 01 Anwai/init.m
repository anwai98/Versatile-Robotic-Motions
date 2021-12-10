% Initalization file for the direct kinematics 

flag_robot = 4;
%
% 1 - planar 2 links
% 2 - planar 3 links
% 3 - antrhopomorphic 3 links
% 4 - jaco2 7 links

switch flag_robot
    case 1
        fprintf('\n planar 2 links \n')
        % parameters
        a     = [1 .4]';
        alpha = zeros(2,1);
        d     = zeros(2,1);
        theta = [0 45/180*pi]';
    case 2
        fprintf('\n planar 3 links \n')
        % parameters
        a     = [1 .4 .2]';
        alpha = zeros(3,1);
        d     = zeros(3,1);
        theta = [0 0 0]';
    case 3
        fprintf('\n antrhopomorphic 3 links \n')
        % parameters
        a     = [0 .4 .2]';
        alpha = [pi/2 0 0]';
        d     = zeros(3,1);
        theta = [0 pi/4 -pi/8]';
    case 4
        fprintf('\n jaco2 7 links \n')
        % parameters
        a     = zeros(7,1);
        alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
        d     = [0.2755 0 -0.410 -0.0098 -0.3072 0 0.25]';
        theta = [pi/2 0 0 pi 0 0 0]';
end

DH = [a alpha d theta];


