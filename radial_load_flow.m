%% backward-forward sweeping, distribution load flow
function [Busvoltage,Linecurrent]=radial_load_flow(bdata,ldata)
    Iterations=1;
    sizl=size(ldata); % Number of lines
    sizb=size(bdata); % Number of buses
    totalbus=sizb(1,1); % Number of buses
    for i=1:totalbus-1
        Terminalbuses(i,1)=0;
        Intermediatebuses(i,1)=0;
    end
    co=0;in=0; 
    for i=1:sizl(1,1)
        for j=1:sizl(1,1)
            if (i~=j)&&(ldata(i,2)==ldata(j,1))
                co=co+1;
            end
        end
        if co==0
            in=in+1;
            Terminalbuses(in,1)=ldata(i,2); % The buses located at end of the feeder
        else
            in=in+1;
            Intermediatebuses(in,1)=ldata(i,2); % non-terminal buses
        end
        co=0;
    end
    %Initializing Voltage and Current matrices
    for i=1:totalbus
        Busvoltage(i,1)=1;
    end
    for i=1:sizl(1,1)
        Linecurrent(i,1)=0;
    end
    for i=1:totalbus-1
        if(Terminalbuses(i,1)~=0)
            Busvoltage(Terminalbuses(i,1),1)=1;
        end
    end
    Busvoltage_old=Busvoltage;
    dV(1:totalbus)=1;   
    while max(dV)>2e-2
        % Backward Sweep
        % calculation of flowing currents to terminal buses
        t=0;int=0;
        for i=1:sizl(1,1)
            for j=1:totalbus-1
                if ldata(i,2)== Terminalbuses(j,1)
                    t=t+1; % checking that the bus is terminal bus or not
                end
            end
            if(t~=0)
                Linecurrent(i,1)=conj(complex(bdata(ldata(i,2),2),bdata(ldata(i,2),3))/Busvoltage(ldata(i,2),1));
            end
            t=0;
        end
        % line current between intermediate buses calculation
        for i=1:sizl(1,1)
            for j=1:totalbus-1
                if ldata(i,2) == Intermediatebuses(j,1)
                    int=int+1; % checking that the bus is intermediate bus or not
                end
            end
            if(int~=0)
                Busvoltage(ldata(i,2),1)=Busvoltage(ldata(i+1,2),1)+((Linecurrent(i+1,1))*(complex(ldata(i+1,3),ldata(i+1,4))));
                a=find(ldata(:,1)==i);
                Outcurrent=0;
                if size(a,1)>=1
                    for ii=size(a,1)
                        Outcurrent=Outcurrent+Linecurrent(ii,1);
                    end
                end
                Linecurrent(i,1)=(conj(complex(bdata(ldata(i,2),2),bdata(ldata(i,2),3))/Busvoltage(ldata(i,2),1)))+Outcurrent;
            end
            int=0;
        end
        % Forward sweep
        Busvoltage(1,1)=1;
        for i=1:sizl(1,1)
            if (ldata(i,1)==1)
                Busvoltage(ldata(i,2),1)=Busvoltage(1,1)-(Linecurrent(i,1)*(complex(ldata(i,3),ldata(i,4))));
            else
                Busvoltage(ldata(i,2),1)=Busvoltage(ldata(i,1),1)-(Linecurrent(i,1)*(complex(ldata(i,3),ldata(i,4))));
            end     
        end
        dV=abs(Busvoltage-Busvoltage_old);
        Busvoltage_old=Busvoltage;
        Iterations=Iterations+1;
    end
return