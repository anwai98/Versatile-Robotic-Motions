% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for the kinova Jaco 2 7-DOF robot
%
% Read Instructions.odt first !
%
% Do not modify any part of this file except the strings within
%    the symbols << >>
%
% G. Antonelli, Introduction to Robotics, spring 2019

function [t, q] = main
close all
clc
clear

addpath('functions_coppelia/');
addpath('functions_matlab/');
porta = 19997;          % default V-REP port
tf = 4;                 % final time
Ts = 0.01;              % sampling time
t  = 0:Ts:tf;           % time vector
N  = length(t);         % number of points of the simulation
n = 7;                  % joint number
q      = zeros(n,N);    % q(:,i) collects the joint position for t(i)
q_jaco = zeros(n,N);    % q_jaco(:,i) collects the joint position for t(i) in Kinova convention
dq     = zeros(n,N);          % q(:,i) collects the joint position for t(i)

q(:,1) = [  
    1.7101
    0.9779
   -0.4966
    2.2657
   -2.7263
    1.7744
    1.0485]'; % approximated home configuration

% <<
% Put here any initialization code: DH table, gains, final position,
% cruise velocity, etc.
% >>

DH_a = [0 0 0 0 0 0 0]';
DH_alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
DH_d = [0.2755 0 -0.41 -0.0098 -0.3111 0 0.2638]';
DH = [DH_a DH_alpha DH_d q(:,1)];

K = diag([50*[1 1 1], 50*[1 1 1]]);

clc
fprintf('----------------------');
fprintf('\n simulation started ');
fprintf('\n trying to connect...\n');
[clientID, vrep ] = StartVrep(porta);
%vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

handle_joint = my_get_handle_Joint(vrep,clientID); % handle to the joints

% main simulation loop
DH(:,4) = q(:,1);
T = DirectKinematics(DH);

R_i = T(1:3,1:3,n);
R_f = [1 0 0; 0 1 0; 0 0 1];
Rif = R_i'*R_f;

axisangle_d = Rot2Axis(Rif);

rad = 0.2;

for i=1:N
    
    %Direct Kinematics
    DH(:,4) = q(:,i);
    T = DirectKinematics(DH);
    x(:,i) = T(1:3,4,n);
    quat(:,i) = Rot2Quat(T(1:3,1:3,n));
    
    [s,~,~] = trapezoidal(0, pi*rad, 0.3, 3, t(i));
    dp_s = [rad*cos(s/rad); -rad*sin(s/rad); 0];
    xd(1:3,i) = ([0.2 -0.4 0.7]') + ([0 1 0; -1 0 0; 0 0 1]*dp_s);
    
    %T.V. for of Orientation
    [theta,~,~] = trapezoidal(0, axisangle_d(4), 1.6, 3, t(i));
    axisangle = [axisangle_d(1:3); theta];
    Rd = Axis2Rot(axisangle);
    Rd = R_i*Rd; %Instantaneous Rotation Rd
    quat_d(:,i) = Rot2Quat(Rd);
    
    %xd(1:3,i) = p_i;
        
    J = Jacobian(DH);
    
    %Inverse Kinematics Algorithm
    error_pos(:,i) = xd(:,i) - x(:,i);
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    error(:,i) = [error_pos(:,i);error_quat(:,i)];
    
    dq(:,i) = pinv_damped(J)*K*error(:,i);
    
    if i<N
        q(:,i+1) = q(:,i) + Ts*dq(:,i);
    end
    % DH -> Kinova conversion
    q_jaco(:,i) = mask_q_DH2Jaco(q(:,i));
    my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,i));
    % q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
    % Kinova conversion -> DH
    % q_act(:,i) = mask_q_Jaco2DH(q_act(:,i));
    
    pause(Ts);
end
figure
subplot(411)
plot(t,q,'linewidth',2)
ylabel('joint position [rad]')
subplot(412)
plot(t,dq,'linewidth',2)
ylabel('joint velocity [rad/s]')
subplot(413)
plot(t,error(1:3,:),'linewidth',2)
ylabel('position error [m]')
subplot(414)
plot(t,error(4:6,:),'linewidth',2)
ylabel('orientation error [-]')
xlabel('time [s]')
%vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);


DeleteVrep(clientID, vrep);

end

% constructor
function [clientID, vrep ] = StartVrep(porta)

vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);        % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);% start the simulation

if (clientID>-1)
    disp('remote API server connected successfully');
else
    disp('failed connecting to remote API server');
    DeleteVrep(clientID, vrep); %call the destructor!
end
% to change the simulation step time use this command below, a custom dt in v-rep must be selected,
% and run matlab before v-rep otherwise it will not be changed
% vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, 0.002, vrep.simx_opmode_oneshot_wait);
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
end

% destructor
function DeleteVrep(clientID, vrep)

vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
%vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
vrep.simxFinish(clientID);  % close the line if still open
vrep.delete();              % call the destructor!
disp('simulation ended');

end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)

[m,n] = size(q);
for i=1:n
    for j=1:m
        err = vrep.simxSetJointPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
        if (err ~= vrep.simx_error_noerror)
            fprintf('failed to send joint angle q %d \n',j);
        end
    end
end

end

function handle_joint = my_get_handle_Joint(vrep,clientID)

[~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_1',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_2',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_3',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_4',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_5',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_6',vrep.simx_opmode_oneshot_wait);
[~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)

[~,n] = size(q);

for i=1:n
    joints_positions = vrep.simxPackFloats(q(:,i)');
    [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);
    
    if (err~=vrep.simx_return_ok)
        fprintf('failed to send the string signal of iteration %d \n',i);
    end
end
pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high

end


function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)

for j=1:n
    vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
end

pause(0.05);

for j=1:n
    [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
end

if (err(j)~=vrep.simx_return_ok)
    fprintf(' failed to get position of joint %d \n',j);
end

end