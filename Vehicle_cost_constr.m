function v = Vehicle_cost_constr(x,Ts,Np,th)
% Function that computes the trajectory of the vehicle model and returns the distance X covered (cost function)
% and the constraints pertaining to the double turn track

%% Build vector of inputs
t_in        =   [0:Ts:(Np-1)*Ts]';
xi0         =   [0;x(1:2,1);0;x(3,1);0];
u_in        =   [x(4:Np+3,1)';
                x(Np+4:end,1)'];

assignin('base','xi0',xi0);
assignin('base','t_in',t_in);
assignin('base','u_in',u_in);

%% Run simulation with FFD
time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

xi_sim      =   zeros(6,Nsim_FFD);
xi_sim(:,1) =   xi0;
for ind=2:Nsim_FFD
    u                   =   u_in(:,1+floor(time_FFD(ind)/Ts));
    xidot               =   vehicle(0,xi_sim(:,ind-1),u,0,th);
    xi_sim(:,ind)       =   xi_sim(:,ind-1)+Ts/Nblock*xidot;
end

X_sim       =   xi_sim(1,1:Nblock:end)';
Y_sim       =   xi_sim(2,1:Nblock:end)';

%% Compute path constraints h(x)
h           =   [Y_sim-(tanh((X_sim-100)/2e1)*10+5);
                -Y_sim+(tanh((X_sim-75)/2e1)*10+15)];

%% Compute cost function f(x)
delta_diff  =   (x(Np+5:end,1)-x(Np+4:end-1,1));
Td_diff     =   (x(5:Np+3,1)-x(4:Np+2,1));
f           =   -X_sim(end,1)+1e3*xi_sim(5,end)^2+1e3*(delta_diff'*delta_diff)+1e-2*(Td_diff'*Td_diff);

%% Stack cost and constraints
v           =   [f;h];
