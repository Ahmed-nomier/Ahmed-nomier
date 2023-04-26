{
%Without GUI
%===========
clc; 
clear;
close all;
%Variables
%=========
A=input('Please Enter the number of sections as most 3 sections = ');
B=[];
if A==1
    B=input('please Enter the type of Second cable (R-C-T-G) ','s');
end
if B=='C'
    A=4;
end
v_i= input('Enter incident voltage =  '); %Input Voltage
z1= input('Enter First impedence =  '); %Impedence of the first line
if A==2 || A==3 || B=='R'
    z2= input('Enter second impedence =  '); %Impedence of the second line
    if A==3
        z3= input('Enter third impedence =  '); %Impedence of the third line
    end
end
if B=='T'
    z2=10e10;
end
if B=='G'
    z2=10e-10;
end
if B=='C'
    z2=input('Enter Capacitance in farad =  ') ; %Impedence of the second line
end
s1= input('Enter First speed in m/s =  '); %Speed 1 in m/s
if A==2 || A==3 
    s2= input('Enter second speed in m/s =  '); %Speed 2 in m/s
    if A==3
        s3= input('Enter third speed in m/s =  '); %Speed 3 in m/s
    end
end
d1= input('Enter First cable distance in m =  '); %length of first line in m
if A==2 || A==3
    d2= input('Enter second cable distance in m =  '); %length of second line in m
    if A==3
       d3= input('Enter third cable distance in m =  '); %length of third line in m 
    end
end
%}
t1=(d1/s1)*1e06; %microseconds
if A==2 || A==3
    t2=(d2/s2)*1e06; %microseconds
    if A==3
      t3=(d3/s3)*1e06; %microseconds 
    end
end

if A<4
%Volt Coefficients
%=================
taw_vA=0; %Transmission Coefficien at A
row_vA=-1; %Reflection Coefficient at A
if A==1 
    taw_vB=(2*z2/(z1+z2));
    row_vB=taw_vB-1;
elseif A>=2
    taw_vBC=(2*z2/(z1+z2)); %Transmission Coefficient at B to C
    row_vBA=taw_vBC-1; %Reflection Coefficien at B to A
    taw_vBA=(2*z1/(z1+z2)); %Transmission Coefficient at B to A
    row_vBC=taw_vBA-1; %Reflection Coefficient at B to C
    if A==2
        taw_vC=2; %Transmission Coefficien at C
        row_vC=1; %Reflection Coefficient at C
    elseif A==3
        taw_vCD=2*z3/(z2+z3); %Transmission Coefficient at C to D
        row_vCB=taw_vCD-1; %Reflection Coefficien at C to B
        taw_vCB=2*z2/(z2+z3); %Transmission Coefficient at C to B
        row_vCD=taw_vCB-1; %Reflection Coefficien at C to D
        taw_vD=2; %Transmission Coefficien at D
        row_vD=1; %Reflection Coefficient at D
    end
end
%Current Coefficients
%====================
taw_iA=2; %Transmission Coefficien at A
row_iA=1; %Reflection Coefficient at A
if A==1
    taw_iB=(2*z1/(z1+z2));
    row_iB=taw_iB-1;
elseif A>=2
    taw_iBC=(2*z1/(z1+z2)); %Transmission Coefficient at B to C
    row_iBA=taw_iBC-1; %Reflection Coefficien at B to A
    taw_iBA=(2*z2/(z1+z2)); %Transmission Coefficient at B to A
    row_iBC=taw_iBA-1; %Reflection Coefficient at B to C
    if A==2
        taw_iC=0; %Transmission Coefficien at C
        row_iC=-1; %Reflection Coefficient at C
    elseif A==3
        taw_iCD=2*z2/(z2+z3); %Transmission Coefficient at C to D
        row_iCB=taw_iCD-1; %Reflection Coefficien at C to B
        taw_iCB=2*z3/(z2+z3); %Transmission Coefficient at C to B
        row_iCD=taw_iCB-1; %Reflection Coefficien at C to D
        taw_iD=0; %Transmission Coefficien at D
        row_iD=-1; %Reflection Coefficient at D
    end
end

v=[v_i,1,2,0,t1]; %[volt on arrow , from bus A , to bus B , time at bus A , time at bus B]
%note: Bus A=1 , Bus B=2 & Bus C=3
i=[v_i/z1,1,2,0,t1]; 
v_A=v_i; v_B=[]; v_C=[]; v_D=[]; i_A=[]; i_B=[]; i_D=0;
if A==2
    i_C=0;
elseif A==3
    i_C=[];
end

for n=1:30
    bus = v(n,3);
    bus_previous=v(n,2);
    if bus==1
        %Reflected from A to B
        v(end+1,:)=[v(n,1)*row_vA ,1,2,v(n,5),v(n,5)+t1];
        i(end+1,:)=[i(n,1)*row_iA ,1,2,i(n,5),i(n,5)+t1];
        %Transmitted from A
        i_A(end+1,:)=[i(n,1)*taw_iA,i(n,5)];
    elseif A==1 && bus==2
        %Reflected from B to A
        v(end+1,:)=[v(n,1)*row_vB ,2,1,v(n,5),v(n,5)+t1];
        i(end+1,:)=[i(n,1)*row_iB ,2,1,i(n,5),i(n,5)+t1];
        %Transmitted from B
        v_B(end+1,:)=[v(n,1)*taw_vB ,v(n,5)];
        i_B(end+1,:)=[i(n,1)*taw_iB,i(n,5)];
    elseif A>=2 && bus_previous==1 && bus==2
        %Transmitted from B to C
        v(end+1,:)=[v(n,1)*taw_vBC ,2,3,v(n,5),v(n,5)+t2];
        i(end+1,:)=[i(n,1)*taw_iBC ,2,3,i(n,5),i(n,5)+t2];
        v_B(end+1,:)=[v(end,1),v(end,4)];
        i_B(end+1,:)=[i(end,1),i(end,4)];
        %Reflected from B to A
        v(end+1,:)=[v(n,1)*row_vBA ,2,1,v(n,5),v(n,5)+t1];
        i(end+1,:)=[i(n,1)*row_iBA ,2,1,i(n,5),i(n,5)+t1];
    elseif A>=2 && bus_previous==3 && bus==2
        %Transmitted from B to A
        v(end+1,:)=[v(n,1)*taw_vBA ,2,1,v(n,5),v(n,5)+t1];
        i(end+1,:)=[i(n,1)*taw_iBA ,2,1,i(n,5),i(n,5)+t1];
        v_B(end+1,:)=[v(end,1),v(end,4)];
        i_B(end+1,:)=[i(end,1),i(end,4)];
        %Reflected from B to C
        v(end+1,:)=[v(n,1)*row_vBC ,2,3,v(n,5),v(n,5)+t2];
        i(end+1,:)=[i(n,1)*row_iBC ,2,3,i(n,5),i(n,5)+t2];
    elseif A==2 && bus==3
        %Reflected from C to B
        v(end+1,:)=[v(n,1)*row_vC ,3,2,v(n,5),v(n,5)+t2];
        i(end+1,:)=[i(n,1)*row_iC ,3,2,i(n,5),i(n,5)+t2];
        %Transfmitted from C
        v_C(end+1,:)=[v(n,1)*taw_vC,v(n,5)];
    elseif A==3 && bus_previous==2 && bus==3
        %Transmitted from C to D
        v(end+1,:)=[v(n,1)*taw_vCD ,3,4,v(n,5),v(n,5)+t3];
        i(end+1,:)=[i(n,1)*taw_iCD ,3,4,i(n,5),i(n,5)+t3];
        v_C(end+1,:)=[v(end,1),v(end,4)];
        i_C(end+1,:)=[i(end,1),i(end,4)];
        %Reflected from C to B
        v(end+1,:)=[v(n,1)*row_vCB ,3,2,v(n,5),v(n,5)+t2];
        i(end+1,:)=[i(n,1)*row_vCB ,3,2,i(n,5),i(n,5)+t2];
    elseif A==3 && bus_previous==4 && bus==3
        %Transmitted from C to B
        v(end+1,:)=[v(n,1)*taw_vCB ,3,2,v(n,5),v(n,5)+t2];
        i(end+1,:)=[i(n,1)*taw_vCB ,3,2,i(n,5),i(n,5)+t2];
        v_C(end+1,:)=[v(end,1),v(end,4)];
        i_C(end+1,:)=[i(end,1),i(end,4)];
        %Reflected from C to D
        v(end+1,:)=[v(n,1)*row_vCD ,3,4,v(n,5),v(n,5)+t3];
        i(end+1,:)=[i(n,1)*row_iCD ,3,4,i(n,5),i(n,5)+t3];
    elseif A==3 && bus==4
       %Reflected from D to C
        v(end+1,:)=[v(n,1)*row_vD ,4,3,v(n,5),v(n,5)+t3];
        i(end+1,:)=[i(n,1)*row_iD ,4,3,i(n,5),i(n,5)+t3];
        %Transfmitted from D
        v_D(end+1,:)=[v(n,1)*taw_vD,v(n,5)]; 
    end
end

%Sorting v_B [volt Leaving B , time]
%========================================
for x=1:length(v_B)
    n=2;
    while n<=length(v_B)
        if v_B(n,2)<v_B(n-1,2)
          v_B([n-1 n],:)=v_B([n n-1],:);%Swaping the 2 rows  
        elseif v_B(n,2)==v_B(n-1,2)
            v_B(n,1)=v_B(n,1)+v_B(n-1,1);
            v_B(n-1,:)=[];
            n=n-1;
        end
        n=n+1;
    end
end
if A>=2
    %Sorting v_C [volt Leaving C , time]
    %========================================
    for x=1:length(v_C)
        n=2;
        while n<=length(v_C)
            if v_C(n,2)<v_C(n-1,2)
              v_C([n-1 n],:)=v_C([n n-1],:);%Swaping the 2 rows  
            elseif v_C(n,2)==v_C(n-1,2)
                v_C(n,1)=v_C(n,1)+v_C(n-1,1);
                v_C(n-1,:)=[];
                n=n-1;
            end
            n=n+1;
        end
    end
end
if A==3
    %Sorting v_D [volt Leaving D , time]
    %========================================
    for x=1:length(v_D)
        n=2;
        while n<=length(v_D)
            if v_D(n,2)<v_D(n-1,2)
              v_D([n-1 n],:)=v_D([n n-1],:);%Swaping the 2 rows  
            elseif v_D(n,2)==v_D(n-1,2)
                v_D(n,1)=v_D(n,1)+v_D(n-1,1);
                v_D(n-1,:)=[];
                n=n-1;
            end
            n=n+1;
        end
    end
end

%Sorting i_A [current Leaving A , time]
%========================================
for x=1:length(i_A)
    n=2;
    while n<=length(i_A)
        if i_A(n,2)<i_A(n-1,2)
          i_A([n-1 n],:)=i_A([n n-1],:);%Swaping the 2 rows  
        elseif i_A(n,2)==i_A(n-1,2)
            i_A(n,1)=i_A(n,1)+i_A(n-1,1);
            i_A(n-1,:)=[];
            n=n-1;
        end
        n=n+1;
    end
end
%Sorting i_B [current Leaving B , time]
%======================================
for x=1:length(i_B)
    n=2;
    while n<=length(i_B)
        if i_B(n,2)<i_B(n-1,2)
          i_B([n-1 n],:)=i_B([n n-1],:);%Swaping the 2 rows  
        elseif i_B(n,2)==i_B(n-1,2)
            i_B(n,1)=i_B(n,1)+i_B(n-1,1);
            i_B(n-1,:)=[];
            n=n-1;
        end
        n=n+1;
    end
end
if A==3
    %Sorting i_C [current Leaving C , time]
    %======================================
    for x=1:length(i_C)
        n=2;
        while n<=length(i_C)
            if i_C(n,2)<i_C(n-1,2)
              i_C([n-1 n],:)=i_C([n n-1],:);%Swaping the 2 rows  
            elseif i_C(n,2)==i_C(n-1,2)
                i_C(n,1)=i_C(n,1)+i_C(n-1,1);
                i_C(n-1,:)=[];
                n=n-1;
            end
            n=n+1;
        end
    end  
end

%Drawing
%=======
for x=1:length(v_B)
        table_vB(1,2)=0;
        table_vB(x+1,1)=v_B(x,1)+table_vB(x,1);
        table_vB(x+1,2)=v_B(x,2);
end
if A>=2
    for x=1:length(v_C)
            table_vC(1,2)=0;
            table_vC(x+1,1)=v_C(x,1)+table_vC(x,1);
            table_vC(x+1,2)=v_C(x,2);
    end
end
if A==3
    for x=1:length(v_D)
        table_vD(1,2)=0;
        table_vD(x+1,1)=v_D(x,1)+table_vD(x,1);
        table_vD(x+1,2)=v_D(x,2);
    end
end
for x=1:length(i_A)
        table_iA(1,2)=0;
        table_iA(x+1,1)=i_A(x,1)+table_iA(x,1);
        table_iA(x+1,2)=i_A(x,2);
end
for x=1:length(i_B)
        table_iB(1,2)=0;
        table_iB(x+1,1)=i_B(x,1)+table_iB(x,1);
        table_iB(x+1,2)=i_B(x,2);
end 
if A==3
   for x=1:length(i_C)
        table_iC(1,2)=0;
        table_iC(x+1,1)=i_C(x,1)+table_iC(x,1);
        table_iC(x+1,2)=i_C(x,2);
   end  
end
    figure
    line([0,100e-06],[v_A,v_A]);
    xlabel('time us')
    ylabel('transmitted voltage at A')
    
    figure
    stairs(table_vB(:,2),table_vB(:,1))
    xlabel('time us')
    ylabel('transmitted voltage at B')
    
    if A>=2
        figure
        stairs(table_vC(:,2),table_vC(:,1))
        xlabel('time us')
        ylabel('transmitted voltage at C')
    end
    
    if A==3
        figure
        stairs(table_vD(:,2),table_vD(:,1))
        xlabel('time us')
        ylabel('transmitted voltage at D') 
    end
    
    figure
    stairs(table_iA(:,2),table_iA(:,1))
    xlabel('time us')
    ylabel('transmitted current at A')
    
    figure
    stairs(table_iB(:,2),table_iB(:,1))
    xlabel('time us')
    ylabel('transmitted current at B')
    
    if A==2
      figure
      line([0,100e-06],[0,0])
      xlabel('time us')
      ylabel('transmitted current at C')
    end
    
    if A==3
       figure
       stairs(table_iC(:,2),table_iC(:,1))
       xlabel('time us')
       ylabel('transmitted current at C') 
    end
    
    if A==3
        figure
        line([0,100e-06],[0,0])
        xlabel('time us')
        ylabel('transmitted current at D')
    end
end 

if A==4
%Coefficients
%============
i_i=v_i/z1;
syms x;
f=x/x;
taw_vB=(2*(laplace(f)*(1/z2)))/(z1+((laplace(f)*(1/z2)))); %Transmission Coefficien at A
row_vB=taw_vB-1; %Reflection Coefficient at A
taw_iB=(2*z1)/(z1+(1/(laplace(f)*(1/z2))));
V_B=laplace(f)*v_i*taw_vB;
V_BT=ilaplace(V_B);
I_B=laplace(f)*i_i*taw_iB;
I_BT=ilaplace(I_B);
%Drawing
%=======
syms t;
figure
fplot(V_BT,[0 0.01])
xlabel('time us')
ylabel('Voltage Transmitted at B')
figure
fplot(I_BT,[0 0.01])
xlabel('time us')
ylabel('Current Transmitted at B')
end


