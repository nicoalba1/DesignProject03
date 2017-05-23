function func = Controller
% INTERFACE
%
%   sensors
%       .theta      (pitch angle)
%       .phi        (elevator angle)
%
%   references
%       .theta      (reference pitch angle)
%
%   parameters
%       .tStep      (time step)
%       .phidotMax  (maximum elevator angular velocity)  
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .phidot     (elevator angular velocity)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [actuators,data] = initControlSystem(sensors,references,parameters,data)

data.A = [0 0 0 0 1;0 0 0 0 0;-8.769 0.80132 -0.015242 -0.16936 0.099377;...
          120.57 24.197 1.4266 -19.902 1.101;-56.413 -80.648 0.82601 9.1778 -5.3854];
     
data.B = [0;1;0.012149;0.19752;-0.6584];
data.C = [1 0 0 0 0;...
          0 1 0 0 0];

data.K = [-855.09 86.716 41.828 -10.139 -163.62];
data.L = [6.0657 -1.7823;-1.7823 2.6122;-1.5521 5.4214;28.716 -4.8763;14.984 -15.466];

data.xdot = 6;
data.zdot = 0;
data.thetadot = 0;

% data.xhat=[0; 0; data.xdot; data.zdot; data.thetadot];
data.xhat=[sensors.theta; sensors.phi; data.xdot; data.zdot; data.thetadot];

data.xe= [0.004613; -0.06604; 6.097; -0.5488; 0];
data.kRef = 1/(data.C*inv(data.A-data.B*data.K)*data.B);

[actuators,data] = runControlSystem(sensors,references,parameters,data);
end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%


function [actuators,data] = runControlSystem(sensors,references,parameters,data)

data.r = [.1*sensors.theta;0];


h=1/100; % Time-Step

y= [sensors.theta; sensors.phi]- data.C*data.xe;
u= -data.K*data.xhat +data.kRef*data.r;

data.xhat=data.xhat+h*(data.A*data.xhat+data.B*u-data.L*(data.C*data.xhat-y));

actuators.phidot = u;
end

