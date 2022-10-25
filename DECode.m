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
%% Initial Values
nvar=5;
m=50;                           % Number of Particles
max_it=200 ;                     % Maximum number of iterations
w_max=0.9; w_min=0.4;           % Inertia weight
c1=1.5; c2=1.5;                 % Social acceleration constant (c1 & c2)
% Initializing variables
Xmin=1; Xmax=size(Line_data,1);
Vmax=Xmax;
Vx=0.5*(-Vmax+2*Vmax*rand(nvar,m));         % Velocity
X=round(Xmin+(Xmax-Xmin).*rand(nvar,m));	% X is considered active power of generators
for i=1:m
    X_init(:,i)=(33:37)';
end
% X=X_init;
[Fitness Loss V_DEV Bus01]=objective_function(X',Bus_data,Line_data,load_senarios,m);
P_old=Fitness;
%%
MontIter=500;
for(ijk=1:1:MontIter)
%     X=round(Xmin+(Xmax-Xmin).*rand(nvar,m));
for iter=1:max_it
    [Fitness Loss V_DEV Bus01]=objective_function(X',Bus_data,Line_data,load_senarios,m);
    b=(Fitness);
    c=sort(b);
for i=1:numel(b)
 for j=1:numel(b)
 if c(i)==b(j)
 d(i)=j;
 end
 end
end

best(1,1:25)=c(1,1:25);
bests(1,1:25)=d(1,1:25);
bad(1,1:25)=c(1,26:50);
bads(1,1:25)=d(1,26:50);
for(j=1:1:15)
    PTOT1=0;
    PTOT2=0;
    Vbest=0;
Sa=size(best);
Saa=Sa(1,1);
Sb=size(bad);
Sbb=Sb(1,2);
Ga1=ceil(rand*Saa);
Ga2=ceil(rand*Saa);
Gb1=ceil(rand*Sbb);
% X=X';
x1=X(:,bests(1,Ga1));
x2=X(:,bests(1,Ga2));
x3=X(:,bads(1,Gb1));
F=0.7;
if(Ga1>=Ga2)
    Vbest=x1+F*(x3-x2);
else 
    Vbest=x1+F*(x2-x3);
end
Vbest=abs(Vbest);
Vbest=ceil(Vbest);
% Vbest=Vbest';
       if(Vbest(1,1)>35)
            Vbest(1,1)=35;
       end
       if (Vbest(1,1)<0)
            Vbest(1,1)=0;
       end
       if(Vbest(2,1)>35)
            Vbest(2,1)=35;
       end
       if (Vbest(2,1)<0)
            Vbest(2,1)=0;
       end
       if(Vbest(3,1)>35)
            Vbest(3,1)=35;
       end
       if (Vbest(3,1)<0)
            Vbest(3,1)=0;
       end
       if(Vbest(4,1)>35)
            Vbest(4,1)=35;
       end
       if (Vbest(4,1)<0)
            Vbest(4,1)=0;
       end
%   Vbest=Vbest';           
%    Vbestcost=abs(feval(costFunction,Vbest));
    Vbestvec(:,j)=Vbest;  
%    Vbestcostvec(j,1)=Vbestcost;
end
% GLOBALBESTCOST(1:10,1)=Vbestcostvec(1:10,1);
%***************************************************************

%create mutation bad vector
for (k=1:1:10)
    Vbad=0;
Sa=size(best);
Saa=Sa(1,2);
Sb=size(bad);
Sbb=Sb(1,1);
Ga1=ceil(rand*Saa);
Gb1=ceil(rand*Sbb);
Gb2=ceil(rand*Sbb);
x11=X(:,bests(1,Ga1));
x22=X(:,bads(1,Gb1));
x33=X(:,bads(1,Gb2));
F=0.7;
if(Gb1>=Gb2)
Vbad=x22+F*(x11-x33);
else
    Vbad=x22+F*(x33-x11);
end
Vbad=abs(Vbad);
Vbad=ceil(Vbad);
         if(Vbad(1,1)>35)
            Vbad(1,1)=35;
        end
        if (Vbad(1,1)<0)
            Vbad(1,1)=0;
        end         
        if(Vbad(2,1)>35)
            Vbad(2,1)=35;
        end     
        if (Vbad(2,1)<0)
            Vbad(2,1)=0;
        end                 
        if(Vbad(3,1)>35)
            Vbad(3,1)=35;
        end         
        if (Vbad(3,1)<0)
            Vbad(3,1)=0;
        end          
        if(Vbad(4,1)>35)
            Vbad(4,1)=35;
        end       
        if (Vbad(4,1)<0)
            Vbad(4,1)=0;
        end         
       
           
%    Vbadcost=abs(feval(costFunction,Vbad));
    Vbadvec(:,k)=Vbad;  
%    Vbadcostvec(k,1)=Vbadcost;
end   

% GLOBALBESTCOST(11:20,1)=Vbadcostvec(1:10,1);

for(i=1:1:25)
Newbestss(:,i)=X(:,bests(1,i));
end
Popnew(:,1:25)=Newbestss;
Popnew(:,26:40)=Vbestvec;
Popnew(:,41:50)=Vbadvec;

% Popnewcost(1:20,1)=best;
% Popnewcost(21:30,1)=Vbestcostvec;
% Popnewcost(31:40,1)=Vbadcostvec;
% 
% GLOBALBESTCOST=sort(GLOBALBESTCOST);
X=Popnew;
 
    
%     disp(['ITERATION : ',num2str(iter),'    FITNESS = ',num2str(best(1,1))]);
    P(1,iter)=best(1,1);
end
%% Outputs
Pg=X(:,1);
% figure
% plot(1:max_it,P,'LineWidth',2.5)
[Fitness Loss V_DEV BUS]=objective_function(Pg',Bus_data,Line_data,load_senarios,1);
[Fitness0 Loss0 V_DEV0 BUS0]=objective_function((33:37),Bus_data,Line_data,load_senarios,1);

LossTotal(1,ijk)=Loss;
V_DEVTotal(1,ijk)= V_DEV;
FittnessTotal(1,ijk)= P(end);
PgTotal(:,ijk)=Pg;
end

loss=0;V_DEV=0;Pg=0;Fit=0;

for(i=1:1:MontIter)

Loss=Loss+ LossTotal(1,i);
V_DEV= V_DEV+ V_DEVTotal(1,i);
Pg=Pg+ PgTotal(:,i);
Fit=Fit+ FittnessTotal(1,i);

end

Loss=Loss/MontIter;
V_DEV=V_DEV/MontIter;
Pg=ceil(Pg/MontIter);
Fit=Fit/MontIter;



% clc
disp('The switches that must be opened  ');
disp(Pg');
disp(['Fitness function value :  ', num2str(Fit)]);
disp(['Total loss value :        ', num2str(Loss)]);
disp(['Voltage deviation index : ', num2str(V_DEV)]);
figure
plot(1:33,abs(BUS0),'--o','LineWidth',2.5);
hold on
plot(1:33,abs(BUS),'--*r','LineWidth',2.5);
save main0.mat Pg P