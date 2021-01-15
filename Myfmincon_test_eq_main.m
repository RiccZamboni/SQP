% Constrained Numerical Optimization for Estimation and Control
% Script to test the constrained optimization algorithm on an 
% example with both equality and inequality constraints

clear all
close all
clc

%% Initialize optimization variables
x0             	=   [-4;-1];

%% Cost function center - (x-c_cost)'*(x-c_cost)
c_cost          =   [-2;1];

%% Nonlinear equality constraint parameters 
a_eq            =   1;

%% Nonlinear inequality constraint parameters
a_ineq          =   2;
b_ineq          =   2;

%% Linear equality constraint parameters
A               =   [];
b               =   [];

%% Linear inequality constraint parameters
C               =   [5 2;5 -2];
d               =   [10;1];

%% Gauss-Newton
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'GN';
myoptions.GN_funF       =	@(x)Simple_test_eq_F(x,c_cost,a_eq,a_ineq,b_ineq);
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.xsequence     =	'on';

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfmincon(@(x)Simple_test_eq(x,c_cost,a_eq,a_ineq,b_ineq),x0,A,b,C,d,1,1,myoptions);

% Plot
x1val           =   linspace(-5,15,1e2);
x2val           =   linspace(-5,10,9e1);
cost_matr_val   =   zeros(9e1,1e2);

for ind1 = 1:1e2
    for ind2 = 1:9e1
        f   =   Simple_test_ineq([x1val(ind1);x2val(ind2)],c_cost,a_ineq,b_ineq);
        cost_matr_val(ind2,ind1)    =   f(1,1);
    end
end

g1val       =   exp(-x1val)+a_eq;
h1val       =   (d(1,1)-C(1,1)*x1val)/C(1,2);
h2val       =   (d(2,1)-C(2,1)*x1val)/C(2,2);
theta       =   linspace(0,2*pi,1e3);
h3valx      =   b_ineq*cos(theta)+a_ineq;
h3valy      =   b_ineq*sin(theta)+a_ineq;
figure(1)
contourf(x1val,x2val,cost_matr_val,20,'k--'),hold on
plot(x1val,g1val,x1val,h1val,...
    x1val,h2val,h3valx,h3valy,xsequence(1,:),xsequence(2,:),'-*','linewidth',1),hold on
axis([-5 6 -5 5])

%% BFGS
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_tkmax      =	1;          
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	0.1;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfmincon(@(x)Simple_test_eq(x,c_cost,a_eq,a_ineq,b_ineq),x0,A,b,C,d,1,1,myoptions);

% Plot
figure(1),plot(xsequence(1,:),xsequence(2,:),'-*')