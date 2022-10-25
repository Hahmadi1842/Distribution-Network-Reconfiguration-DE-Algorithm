function [Fitness,LOSS,V_DEV,Bus00]=objective_function(x,Buses,Lines,senarios,M)
    Bus0=Buses;
    Line0=Lines;
    Loss1=zeros(size(senarios,1),1);
    Voltage_dev1=zeros(size(senarios,1),1);
    for m=1:M
        Buses=Bus0;
        Lines=Line0;
        for i=1:size(x,2)
            ii=1; LL=size(Lines,1);
            while ii<=LL
                if (Lines(ii,1)==Line0(x(m,i),1)) && (Lines(ii,2)==Line0(x(m,i),2))
                    Lines(ii,:)=[];
                    ii=ii-1;
                end
                ii=ii+1; 
                LL=size(Lines,1);
            end
        end
        [B C]=sort(Lines(:,2));
        Lines=Lines(C,:);
        A=zeros(size(Lines,1),size(Buses,1));
        for i=1:size(Lines,1)
            A(i,Lines(i,1))=-1;
            A(i,Lines(i,2))=1;
        end
        A(:,1)=[];
        if size(A,1)==size(A,2)
            redial_index=det(A);
        else
            redial_index=0;
        end
        if redial_index~=0
            for s=1:size(senarios,1)
                Buses=Bus0;
                Buses(:,2:3)=Buses(:,2:3)*senarios(s,1);
                [Bus Line]=radial_load_flow(Buses,Lines);
                % Losses
                Loss=zeros(32,1);
                for jj=1:32
                    Loss(jj)=Lines(jj,3)*(abs(Line(jj)))^2;
                end
                % Voltage Deviation
                Voltage_dev=zeros(33,1);
                for jj=1:33
                    Voltage_dev(jj)=1-abs(Bus(jj));
                end
                Loss1(s)=sum(Loss);
                Voltage_dev1(s)=sum(Voltage_dev);
                Fit(s)=senarios(s,2)*(Loss1(s)+Voltage_dev1(s))*100;
            end
            LOSS(m)=sum(Loss1);
            V_DEV(m)=sum(Voltage_dev1);
            Fitness(m)=sum(Fit);
            Bus00(:,m)=Bus;
        else
            LOSS(m)=1e3;
            V_DEV(m)=1e3;
            Fitness(m)=1e3;
            Bus00(:,m)=1e3*ones(33,1);
        end
    end
return
