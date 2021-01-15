function [xidot,F]=vehicle(t,xi,u,d,th)
% Nonlinear dynamic model of a road vehicle with six states: 2-D position,
% 2-D velocity, yaw angle and yaw rate. Nonlinear lateral tyre forces with
% Fiala model; saturation on braking and driving torque, saturation on 
% front wheel steering angle. Partial de-coupling of longitudinal and
% lateral dynamics (assume longitudinal speed varies slowly with respect to
% lateral dynamics)
%
% Inputs:   t       (time - for use with ode45)
%           xi      (model state)
%           u       (braking/driving torque and steering angle)
%           d       (lateral wind),
%           
%           th      (model parameters)
%
% Outputs:  xi_dot  (derivative of the state with respect to time)
%           F       (longitudinal and lateral forces)

%% Read parameters, states and inputs
% Parameters
m       =       th(1,1);     % vehicle mass (kg)
Jz      =       th(2,1);     % vehicle moment of inertia (kg*m^2)
a       =       th(3,1);     % distance between center of gravity and front axle (m)
b       =       th(4,1);     % distance between center of gravity and rear axle (m)
Cf      =       th(5,1);     % front axle cornering stiffness (N/rad)
Cr      =       th(6,1);     % rear axle cornering stiffness (N/rad)
rw      =       th(7,1);     % wheel radius (m)
mu      =       th(8,1);     % road friction coefficient
Af      =       th(13,1);    % vehicle front surface (m^2)
Al      =       th(14,1);    % vehicle lateral surface (m^2)
Cx      =       th(15,1);    % vehicle front aerodynamic drag coefficient
Rr      =       th(16,1);    % rolling resistance coefficient(N*s/m)
rho     =       th(17,1);    % air density (kg/m^3)

% States
X       =       xi(1,1);    % inertial X position (m)
Y       =       xi(2,1);    % inertial Y position (m)
Ux      =       xi(3,1);    % body x velocity (m/s)
beta    =       xi(4,1);    % sideslip angle (rad)
psi     =       xi(5,1);    % yaw angle (rad)
r       =       xi(6,1);    % yaw rate (rad/s)

% Inputs
Td      =       u(1,1);     % driving/braking torque (N*m)
delta   =       u(2,1);     % front wheel steering angle (rad)
W       =       d(1,1);     % lateral wind speed (m/s)

%% Compute lateral and longitudinal tyre forces
Fzf     =       m*b*9.81/(a+b);             % vertical force on front axle
Fzr     =       m*a*9.81/(a+b);             % vertical force on rear axle
Uy      =       Ux*tan(beta);               % body y velocity (m/s)
alphaf  =       atan2((Uy+r*a),Ux)-delta;    % front slip angle (rad)         
zf      =       tan(alphaf);                % tangent of front slip angle
zr      =       (Uy-r*b)/Ux;                % tangent of rear slip angle
Fyf     =       min(mu*Fzf,max(-mu*Fzf,-Cf*zf+Cf^2*abs(zf)*zf/(3*mu*Fzf)-Cf^3/(27*mu^2*Fzf^2)*zf^3));   % Front lateral force (N)
Fyr     =       min(mu*Fzr,max(-mu*Fzr,-Cr*zr+Cr^2*abs(zr)*zr/(3*mu*Fzr)-Cr^3/(27*mu^2*Fzr^2)*zr^3));   % Rear lateral force (N)
Fx      =       Td/rw;                      % Longitudinal driving/braking force (N)
Fr      =       Ux*Rr;                      % Rolling resistance (N)
Fxd     =       0.5*rho*Cx*Af*Ux^2;         % Front aerodynamic resistance (N)
Fyd     =       0.5*rho*Al*W^2;             % Lateral force due to wind (N)
F       =       [Fyf;Fyr;Fx;Fr;Fxd;Fyd];

% Model equations
xidot(1,1)  =   Ux*cos(psi)-Uy*sin(psi);
xidot(2,1)  =   Ux*sin(psi)+Uy*cos(psi);
xidot(3,1)  =   (Fx-Fyf*sin(delta)-Fr-Fxd)/m;
xidot(4,1)  =   (Fyf*cos(delta)+Fyr+Fyd)/(m*Ux)-r;
xidot(5,1)  =   r;
xidot(6,1)  =   (a*Fyf*cos(delta)-b*Fyr)/Jz;
