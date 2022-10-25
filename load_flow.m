clc;clear all 
load net_data
Vb=12.6e3; Sb=100e3; Zb=Vb^2/Sb;
Bus_data(:,2:3)=1.5*Bus_data(:,2:3)/Sb;
Line_data(:,3:4)=Line_data(:,3:4)/Zb;
[Buses Lines]=radial_load_flow(Bus_data,Line_data);
Bus(:,1)=1:33;
Bus(:,2)=abs(Buses);
Bus(:,3)=angle(Buses)*180/pi;
disp(Bus)