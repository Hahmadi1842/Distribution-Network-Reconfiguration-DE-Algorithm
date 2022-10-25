%% PSO algorithm for reconfiguration 
clc
close all
clear all
load net_data
load load_senario
load_index=1.4;
Vb=12.6e3; Sb=100e3; Zb=Vb^2/Sb;
Bus_data(:,2:3)=load_index*Bus_data(:,2:3)/Sb;
Line_data(:,3:4)=Line_data(:,3:4)/Zb;
[BUS LINE]=radial_load_flow(Bus_data,Line_data(1:32,:));
nvar=9;
m=50;                           % Number of Particles
max_it=200;                     % Maximum number of iterations
w_max=0.9; w_min=0.4;           % Inertia weight
c1=1.7; c2=1.5;                 % Social acceleration constant (c1 & c2)
%%
load main0
% Pg=G_best(:,1);
figure
% plot(1:max_it,P,'LineWidth',2.5)
[Fitness Loss V_DEV BUS]=objective_function(Pg',Bus_data,Line_data,load_senarios,1);
[Fitness0 Loss0 V_DEV0 BUS0]=objective_function((33:37),Bus_data,Line_data,load_senarios,1);
clc
disp('The switches that must be opened  ');
disp(Pg');
% disp(['Fitness function value :  ', num2str(P(end))]);
disp(['Total loss value :        ', num2str(Loss)]);
disp(['Voltage deviation index : ', num2str(V_DEV)]);
figure
plot(1:33,abs(BUS0),'--o','LineWidth',2.5);
hold on
plot(1:33,abs(BUS),'--*r','LineWidth',2.5);
save main0.mat Pg