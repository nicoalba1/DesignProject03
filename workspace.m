% clear all 
% clc
% 

syms theta phi xdot zdot thetadot phidot
load('data.mat')
load('DesignProblem03_EOMs.mat') % load equations of motion

f = symEOM.f;
state = [theta phi xdot zdot thetadot]; % define state

[theta_e phi_e xdot_e zdot_e] = GetEquilibriumPoint(f, 0, 0, 6,0);% how close to initial x dot?
% can change if this doesnt fly very far! just finds eq pt around 0 0 10 0
% can't have xdot be zero- statrts with initial x dot

eqstate = [theta_e phi_e xdot_e zdot_e 0]; % define equilibrium state
eqinput = [0]; % define equilibrium input %% wait why?

% phi dot, theta dot = 0 for equilibrium point
input = [phidot];  % able to control elevator pitch

% fsys = d([state])/dt
% f for the entire system
fsys = [thetadot; phidot; f]; % make sure matches with order of state

% double check for equilibrium, should be close to zero
check_equilibrium = double(subs(fsys,[state input],[eqstate eqinput]));


% calculate A & B matrix:
a= jacobian(fsys, state);
b= jacobian(fsys,input);
A= double(subs(a,[state input],[eqstate eqinput]));
B= double(subs(b,[state input],[eqstate eqinput]));

% From controllerdata.sensors
% y = [theta;phi]= [ 1*(theta), 0*phi, 0*x_dot, 0*zdot, 0*thetadot]
C= [1 0 0 0 0;...
    0 1 0 0 0];

D= [0];

%Check if Controllable
length(A)==rank(ctrb(A,B));
  

%Observer Design

Qc= 1.8*[200 0 0 0 0; ...
     0 1 0 0 0; ...
     0 0 10 0 0; ...
     0 0 0 100 0; ...
     0 0 0 0 140];
Rc= .01;
K=lqr(A,B,Qc,Rc);

Qo= .01*eye(2);         
Ro= .1*...
    [1 0 0 0 0; ...
     0 1 0 0 0; ...
     0 0 1 0 0; ...
     0 0 0 1 0; ...
     0 0 0 0 10];
L=lqr(A',C',Ro,Qo)';

k=mat2str(K,5)
l=mat2str(L,5)


%% 
clear all
close all
clc

n=1000;
x=zeros(n,1);
t=zeros(n,1);

for i=1:n
    DesignProblem03('Controller','datafile','data.mat','display',false);
    load('data.mat');
    x(i)= processdata.x(end);
    t(i)= processdata.t(end);
    figure(1)
    plot(processdata.x,processdata.z)
    grid on
    hold on
    i

end

figure(1)
x_label('Time')
ylabel('Height')
title('Flight Paths')
X=sum(x);
T=sum(t);
t_avg= T/length(t);

x_max=max(x)
x_avg= mean(x)
x_med= median(x)
s=std(x)

%%

nbins= 50;
figure(2)
hist(x,nbins)
line([x_avg,x_avg],[0,90],'linewidth',2,'Color','b')
line([x_med,x_med],[0,90],'linewidth',2,'Color','g')
xlabel('Distance')
ylabel('Instances')
legend('Distance Flown','Average Distance','Median Distance','Location','northwest')
title('Histogram of Distances Flown- Nico Alba')
grid on
grid minor



%%
j=1;
for i=1:length(x)
    if x(i)>10
        z(i)=x(i);
        x_new(j)=z(i);
        j=j+1;
    else
        z(i)=nan;
    end
end

x_new_max= max(x_new)
x_new_avg= mean(x_new)
x_new_med= median(x_new)
s_new=std(x_new)
figure(3)
hist(x_new,50)
line([x_new_avg,x_new_avg],[0,70],'linewidth',2,'Color','b')
line([x_new_med,x_new_med],[0,70],'linewidth',2,'Color','g')
xlabel('Distance')
ylabel('Instances')
legend('Distance Flown','Average Distance','Median Distance','Location','northwest')
title('Histogram of Distances Flown- No Outliers')
grid on
grid minor
