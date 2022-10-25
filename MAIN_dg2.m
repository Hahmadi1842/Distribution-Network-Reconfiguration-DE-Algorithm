%% PSO algorithm for reconfiguration 
clc; close all; clear all
load net_data 
load load_senario
load_index=1.3;
Vb=12.6e3; Sb=100e3; Zb=Vb^2/Sb;
Bus_data(:,2:3)=load_index*Bus_data(:,2:3)/Sb;
Line_data(:,3:4)=Line_data(:,3:4)/Zb;
[BUS LINE]=radial_load_flow(Bus_data,Line_data(1:32,:));
%% Initial Values
nvar=9;
m=50;                           % Number of Particles
max_it=200;                     % Maximum number of iterations
w_max=0.9; w_min=0.4;           % Inertia weight
c1=1.7; c2=1.5;                 % Social acceleration constant (c1 & c2)
% Initializing variables
Xmin=1; Xmax=size(Line_data,1);
Vmax=Xmax;
Vx=(-Vmax+2*Vmax*rand(nvar,m));        % Velocity
X=round(Xmin+(Xmax-Xmin).*rand(nvar,m));   % X is considered active power of generators
for i=1:m
    X_init(:,i)=(33:37)';
end
% X=X_init;
[Fitness Loss V_DEV Bus01]=objective_function_dg2(X',Bus_data,Line_data,load_senarios,m);
P_old=Fitness;
%% Unit Commitment
for iter=1:max_it
    [Fitness Loss V_DEV Bus01]=objective_function_dg2(X',Bus_data,Line_data,load_senarios,m);
    % finding P_best abd G_best
    for j=1:m
        P_old(j)=min(Fitness(j),P_old(j));
        if Fitness(j)<=P_old(j)
            P_best(:,j)=X(:,j);
        end
    end
    k=min(P_old);
    r1=find(P_old<=k);
    r=r1(1);
    P(iter)=P_old(r(1));
    for j=1:m
        G_best(:,j)=P_best(:,r);
    end
    r1=rand(1); r2=rand(1);
    w=w_max-((w_max-w_min)/max_it)*iter;
    X_old=X;
    Vx_new=w*Vx+c1*r1*(P_best-X)+c2*r2*(G_best-X);
    Vx=Vx_new;
    for i=1:nvar
        for j=1:m
            if Vx_new(i,j)>Vmax
                Vx_new(i,j)=Vmax;
            elseif Vx_new(i,j)<-Vmax
                Vx_new(i,j)=-Vmax;
            end
        end
    end
    r3=rand(nvar,m);
    X=round(X+Vx_new);
    for i=1:nvar
        for j=1:m
            if X(i,j)>Xmax
                X(i,j)=Xmax;
            elseif X(i,j)<Xmin
                X(i,j)=Xmin;
            end
        end
    end
    disp(['ITERATION : ',num2str(iter),'    FITNESS = ',num2str(P(end))]);
end
%% Outputs
Pg=G_best(:,1);
figure
plot(1:max_it,P,'LineWidth',2.5)
[Fitness Loss V_DEV BUS]=objective_function_dg2(Pg',Bus_data,Line_data,load_senarios,1);
[Fitness0 Loss0 V_DEV0 BUS0]=objective_function_dg2([(33:37) 1 0 1 0],Bus_data,Line_data,load_senarios,1);
clc
disp('The switches that must be opened  ');
disp(Pg(1:5)');
disp(' DG location and size  ');
Pg(6)=round(Pg(6)*33/37);
Pg(8)=round(Pg(8)*33/37);
Pg(7)=round(Pg(7)*400/37);
Pg(9)=round(Pg(9)*400/37);
disp(Pg(6:7)')
disp(Pg(8:9)')
disp(['Fitness function value :  ', num2str(P(end))]);
disp(['Total loss value :        ', num2str(Loss)]);
disp(['Voltage deviation index : ', num2str(V_DEV)]);
figure
plot(1:33,abs(BUS0),'--o','LineWidth',2.5);
hold on
plot(1:33,abs(BUS),'--*r','LineWidth',2.5);
save main2.mat Pg P