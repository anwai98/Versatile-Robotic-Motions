% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for the kinova Jaco 2 7-DOF robot
% Read Instructions.odt first !
% Do not modify any part of this file except the strings within
%    the symbols << >>
% G. Antonelli, Introduction to Robotics, spring 2019

function [t, q, q_act] = main_template
clear
close all
clc

%init

algorithm = 'inverse';

if strcmp(algorithm,'inverse')
    K = diag([10*[1 1 1], 5*[1 1 1]]);
    fprintf('\n algorithm: inverse of the jacobian \n')
else
    K = diag([50*[1 1 1], 30*[1 1 1]]);
    fprintf('\n algorithm: transpose of the jacobian \n')
end

addpath('functions_coppelia/');
addpath('functions_matlab/');

porta = 19997;          % default V-REP port
tf = 4;                % final time
Ts = 1e-3;            % sampling time
t  = 0:Ts:tf;           % time vector
N  = length(t);         % number of points of the simulation
n = 7;                  % joint number
xd  = zeros(3,N);
x   = zeros(3,N);
q      = zeros(n,N);    % q(:,i) collects the joint position for t(i)
q_jaco = zeros(n,N);    % q_jaco(:,i) collects the joint position for t(i) in Kinova convention
dq     = zeros(n,N);    % q(:,i) collects the joint position for t(i)
quat = zeros(4,N);
quat_d = zeros(4,N);
error_pos = zeros(3,N);
error_quat = zeros(3,N);
error = zeros(6,N);
%q(:,1) = [-90 180 0 180 0 180 0]'/180*pi;   % initial configuration
%q(:,1) = [76.9301  -17.1094 0.1103  43.3456  -94.4318  77.1818 71.7273]'/180*pi; % home configuration
q(:,1) = [1.3439 -0.2967 0 0.7505 -1.6406 1.3439 1.2392]'; % approximated home configuration
q_jaco(:,1) = mask_q_DH2Jaco(q(:,1));

% <<
%
% Put here any initialization code: DH table, gains, final position,
% cruise velocity, etc.
%
% >>

DH_a = [0 0 0 0 0 0 0]';
DH_alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
DH_d = [0.2755 0 -0.41 -0.0098 -0.3072 0 0.25]';
DH = [DH_a DH_alpha DH_d q(:,1)];

K = diag([1*[1 1 1], 5*[1 1 1]]);

clc
fprintf('----------------------');
fprintf('\n simulation started ');
fprintf('\n trying to connect...\n');
[clientID, vrep ] = StartVrep(porta);
%vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

handle_joint = my_get_handle_Joint(vrep,clientID); % handle to the joints
my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,1)); % first move to q0
%q_act(:,1) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
% Kinova conversion -> DH
%q_act(:,1) = mask_q_DH2Jaco(q_act(:,1));

% main simulation loop
for i=1:N
    % <<
    % Put here :
    %   computation of the desired trajectory in the operative space
    %   computation of the error
    %   computation of the desired joint velocity: dq(:,i)
    % >>
    % desired task trajectory
    xd(:,i) = [-0.1 -0.6 0.7]';
    beta = pi;
    Rd = [cos(beta) -sin(beta) 0;
          sin(beta) cos(beta) 0;
           0 0 1];

    quat_d(:,i) = Rot2Quat(Rd);
    % quat_d(:,i) = [0 0 0 1]';
    % direct kinematics
    DH(:,4) = q(:,i);
    T = DirectKinematics(DH);
    x(:,i) = T(1:3,4,n);
    quat(:,i) = Rot2Quat(T(1:3,1:3,n));
    % Jacobian
    J = Jacobian(DH);
    % Inverse kinematics algorithm
    error_pos(:,i) = xd(:,i) - x(:,i);
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    error(:,i) = [error_pos(:,i); error_quat(:,i)];
        
    dq(:,i) = pinv_damped(J)*K*error(:,i); %Joint Velocity
    
    %DK; e; J; IK
   
    % next line to be commented when running his own code
    % dq(:,i) = 1*sin((2*pi/2)*t(i));
    % integration
    % integration
    if strcmp(algorithm,'transpose')
        dq(:,i) = J'*K*error(:,i);
    else
        dq(:,i) = pinv_damped(J)*K*error(:,i);
    end
    
    if i<N
        q(:,i+1) = q(:,i) + Ts*dq(:,i); %Euler Integration - Joint Configuration
    end
    
    % DH -> Kinova conversion
    q_jaco(:,i) = mask_q_DH2Jaco(q(:,i));
    my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,i));
    %q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
    % Kinova conversion -> DH
    %q_act(:,i) = mask_q_Jaco2DH(q_act(:,i));
    
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

figure
DH(:,4) = q(:,N)
DrawRobot(DH);

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

