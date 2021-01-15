% Constrained Numerical Optimization for Estimation and Control
% Script to test the constrained optimization algorithm on a Finite Horizon
% Optimal Control Problem for vehicle trajectory optimization

clear all
close all
clc

%% Model parameters
m       =       1715;               % vehicle mass (kg)
Jz      =       2700;               % vehicle moment of inertia (kg*m^2)
a       =       1.07;               % distance between center of gravity and front axle (m)
b       =       1.47;               % distance between center of gravity and rear axle (m)
Cf      =       95117;              % front axle cornering stiffness (N/rad)
Cr      =       97556;              % rear axle cornering stiffness (N/rad)
rw      =       0.303;              % wheel radius (m)
mu      =       1;                  % road friction coefficient
Tdmax   =       1715*1.7*0.303;     % maximum driving torque (N*m)
Tdmin   =       -1715*9.81*0.303;   % maximum braking torque (N*m)
dmax    =       35*pi/180;          % maximum steering angle (rad)
dmin    =       -35*pi/180;         % minimum steering angle (rad)
Af      =       1.9;                % vehicle front surface (m^2)
Al      =       3.2;                % vehicle lateral surface (m^2)
Cx      =       0.4;                % vehicle front aerodynamic drag coefficient
Rr      =       0.016*m*9.81/30;    % rolling resistance coefficient(N*s/m)
rho     =       1.2;                % air density (kg/m^3)
th      =       [m;Jz;a;b;Cf;Cr;rw;mu;Tdmax;Tdmin;dmax;dmin;Af;Al;Cx;Rr;rho];

%% FHOCP parameters - single shooting
Ts      =       0.5;                % seconds, input sampling period
Tend    =       10;                 % seconds, terminal time
Np      =       Tend/Ts;            % prediction horizon

%% Initialize optimization variables
x0      =       [0;80/3.6;0;        % initial state: Y(m),speed(m/s),psi(rad)
                100*ones(Np,1);     % Torque (Nm),
                zeros(Np,1)];       % Steering angle (rad)

%% Constraints
% Bounds on input variables
C       =       [-eye(2*Np+3)
                eye(2*Np+3)];
d       =       [-5;-150/3.6;-pi/3;
                 -Tdmax*ones(Np,1);
                 -dmax*ones(Np,1);
                 -5;50/3.6;-pi/3;
                 Tdmax*ones(Np,1)/10;
                 dmin*ones(Np,1)];
             
% Number of nonlinear inequality constraints (track borders)
q       =       2*Np;

%% Solution -  BFGS
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';
myoptions.outputfcn     =   @(x)Vehicle_traj(x,Ts,Np,th);

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfmincon(@(x)Vehicle_cost_constr(x,Ts,Np,th),x0,[],[],C,d,0,q,myoptions);

%% Visualize results
[xi_sim] = Vehicle_traj(x,Ts,Np,th);
