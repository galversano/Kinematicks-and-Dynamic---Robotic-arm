clear all
clc
close all


TIME=10;
dt=0.2;

acc_time=0.2*TIME;
dcc_time=1*acc_time;
v_ss_time=TIME-acc_time-dcc_time;
%%%get initials valuse

%Initial and final location
ds=input('enter start piont, like [X;Y;Z]:')
dg=input('enter end piont, like [X;Y;Z]:')

%Roll pitch yaw (start)
 roll=input('enter start roll:') 
 pitch=input('enter start pitch:'); 
 yaw=input('enter start yaw:') 
 
%calculate the transformation matrix by roll pitch yaw (Start)
R_x=[1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
R_y=[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
R_z=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];

Rs= R_z*R_y*R_x;

 

%Roll pitch yaw (end)
 roll=input('enter end roll:') 
 pitch=input('enter end pitch:') 
 yaw=input('enter end yaw:') 
 

%calculate the transformation matrix by roll pitch yaw (End)
R_x=[1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
R_y=[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
R_z=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];

Rg= R_z*R_y*R_x;

    %Inverse_kinematics for start point
    
    %calculate l1 DOF
    l1=ds(3)-800-150*Rs(3,2)
    C=sqrt((ds(1)-150*Rs(1,2))^2+(ds(2)-150*Rs(2,2))^2)
    %calculate l2 DOF
    l2=(C-150)
    
    %calculate theta1 DOF
    theta1=atan2(-1*(ds(1)-150*Rs(1,2))/(l2+150),(ds(2)-150*Rs(2,2))/(l2+150))
    
    A1=[cos(theta1) -sin(theta1) 0 0 ; sin(theta1) cos(theta1) 0 0 ;0 0 1 800 ; 0 0 0 1];
    A2=[1 0 0 0;0 1 0 0; 0 0 1 l1; 0 0 0 1];
    A3=[1 0 0 0;0 1 0 l2; 0 0 1 0; 0 0 0 1];

    R1=A1(1:3,1:3);
    R2=A2(1:3,1:3);
    R3=A3(1:3,1:3);
    %R4=A4(1:3,1:3);

R0_3=R1*R2*R3;
Rtr=transpose(R0_3)*Rs;
%calculate theta3 DOF
theta3=atan2(sqrt(Rtr(2,1)^2+Rtr(2,3)^2),Rtr(2,2)); %Top or bottom arm

%calculate theta2 DOF
theta2=atan2(Rtr(3,2)/sin(theta3),-Rtr(1,2)/sin(theta3));
    
%calculate theta4 DOF
theta4=atan2(Rtr(2,3)/sin(theta3), Rtr(2,1)/sin(theta3));

%in case theta3=0 atan2=inf 
if theta3==0
    theta4=0;
    theta2=0;
end%Top or bottom arm

A4=[cos(theta2) 0 sin(theta2) 0; 0 1 0 0; -sin(theta2) 0 cos(theta2) 0; 0 0 0 1];
A5=[1 0 0 0; 0 1 0 150; 0 0 1 0; 0 0 0 1];

A6=[cos(theta3) -sin(theta3) 0 0 ; sin(theta3) cos(theta3) 0 0 ; 0 0 1 0 ; 0 0 0 1 ];
A7=[1 0 0 0 ; 0 1 0 150 ; 0 0 1 0; 0 0 0 1];
A8=[cos(theta4) 0 sin(theta4) 0 ; 0 1 0 0 ; -sin(theta4) 0 cos(theta4) 0; 0 0 0 1];


%result of inv kinematicks (start point)
q_s=[theta1 l1 l2 theta2 theta3 theta4];


    %Inverse_kinematics for the end point
    
    %calculate l1 DOF
    l1=dg(3)-800-150*Rg(3,2)
    C=sqrt((dg(1)-150*Rg(1,2))^2+(dg(2)-150*Rg(2,2))^2)
    
    %calculate l2 DOF
    l2=(C-150)
   

    %calculate theta1 DOF
    theta1=atan2(-1*(dg(1)-150*Rg(1,2))/(l2+150),(dg(2)-150*Rg(2,2))/(l2+150))
    
   A1=[cos(theta1) -sin(theta1) 0 0 ; sin(theta1) cos(theta1) 0 0 ;0 0 1 800 ; 0 0 0 1];
   A2=[1 0 0 0;0 1 0 0; 0 0 1 l1; 0 0 0 1];
   A3=[1 0 0 0;0 1 0 l2; 0 0 1 0; 0 0 0 1];

    R1=A1(1:3,1:3);
    R2=A2(1:3,1:3);
    R3=A3(1:3,1:3);

R0_3=R1*R2*R3;
Rtr=transpose(R0_3)*Rg;

    %calculate theta3 DOF
theta3=atan2(sqrt(Rtr(2,1)^2+Rtr(2,3)^2),Rtr(2,2)); %Top or bottom arm

    
    %calculate theta2 DOF
theta2=atan2(Rtr(3,2)/sin(theta3),-Rtr(1,2)/sin(theta3));
    
    %calculate theta4 DOF
theta4=atan2(Rtr(2,3)/sin(theta3), Rtr(2,1)/sin(theta3));

%in case theta3=0 atan2=inf 
if theta3==0
    theta4=0;
    theta2=0;
end%Top or bottom arm

A4=[cos(theta2) 0 sin(theta2) 0; 0 1 0 0; -sin(theta2) 0 cos(theta2) 0; 0 0 0 1];
A5=[1 0 0 0; 0 1 0 150; 0 0 1 0; 0 0 0 1];
A6=[cos(theta3) -sin(theta3) 0 0 ; sin(theta3) cos(theta3) 0 0 ; 0 0 1 0 ; 0 0 0 1 ];
A7=[1 0 0 0 ; 0 1 0 150 ; 0 0 1 0; 0 0 0 1];
A8=[cos(theta4) 0 sin(theta4) 0 ; 0 1 0 0 ; -sin(theta4) 0 cos(theta4) 0; 0 0 0 1];

%result of inv kinematicks (end point)
q_t=[theta1 l1 l2 theta2 theta3 theta4];


delta=abs(q_t-q_s);
v_ss=2*delta/(TIME+v_ss_time);
acc=v_ss/acc_time;
dcc=-v_ss/dcc_time;
motion=zeros((TIME/dt),7);
velos=motion;

%for ten kilogram mass
 m=10;
 
 %gravity
 g=9.81; 
    
    
    
for ii=1:6
    e=1;
    
      x_0=0;  
  %acc for every DOF
  for t=0:dt:acc_time
      
    motion(e,ii)=0.5*acc(ii)*t^2+x_0;
    motion(e,7)=t;
    velos(e,ii)=acc(ii)*t;
   e=e+1;
  end
  
  x_0=motion(e-1,ii);
     ee=0; 
     %steady state for every DOF
    for t=dt:dt:v_ss_time
      
    motion(e+ee,ii)=x_0+v_ss(ii)*t;
    motion(e+ee,7)=t+acc_time;
    velos(e+ee,ii)=v_ss(ii);
   ee=ee+1;
  end
   
  x_0=motion(e+ee-1,ii);
     eee=0; 
       %dcc for every DOF
    for t=dt:dt:dcc_time
      
    motion(e+ee+eee,ii)=x_0+v_ss(ii)*t+0.5*dcc(ii)*t^2;
    motion(e+ee+eee,7)=t+acc_time+v_ss_time;
    velos(e+ee+eee,ii)=v_ss(ii)+dcc(ii)*t;
   eee=eee+1;
  end
    
end

%create a matrix for position for every DOF 
for ii=1:6
    if q_s(ii)<q_t(ii)
pos(:,ii)=q_s(ii)*ones(TIME/dt+1,1)+motion(:,ii);
    else
   pos(:,ii)=q_s(ii)*ones(TIME/dt+1,1)-motion(:,ii);
    end
    
end
pos(:,7)=motion(:,7);


matrix=zeros(length(pos),3);
for k=1:length(pos)
    A1=[cos(pos(k,1)) -sin(pos(k,1)) 0 0 ; sin(pos(k,1)) cos(pos(k,1)) 0 0; 0 0 1 800; 0 0 0 1];
    A2=[1 0 0 0; 0 1 0 0; 0 0 1 (pos(k,2)); 0 0 0 1];
    A3=[1 0 0 0; 0 1 0 (pos(k,3)); 0 0 1 0; 0 0 0 1];   
    A4=[cos(pos(k,4)) 0 -sin(pos(k,4)) 0; 0 1 0 150; -sin(pos(k,4)) 0 cos(pos(k,4)) 0; 0 0 0 1]; 
    A5=[cos(pos(k,5)) -sin(pos(k,5))  0  -150*sin(pos(k,5)); sin(pos(k,5)) cos(pos(k,5)) 0 150*cos(pos(k,5)); 0 0 1 0; 0 0 0 1]; 
    A6=[cos(pos(k,6)) 0 sin(pos(k,6)) 0; 0 1 0 0; -sin(pos(k,6)) 1 cos(pos(k,6)) 0; 0 0 0 1];
    
    An=A1*A2*A3*A4*A5*A6*[0;0;0;1]
    T_An=transpose(An(1:3));
    matrix(k,:)=T_An
    
end
%4.1.10.2.6
%theta2 motor
for ii=1:length(pos)
    tor(ii,1)=0;
     tor(ii,2)=10^-3*150*m*g*cos(pos(ii,4))*sin(pos(ii,5));     
end

%theta3 motor
for ii=1:length(pos)
    tor(ii,1)=0;
     tor(ii,3)=10^-3*150*m*g*sin(pos(ii,4))*cos(pos(ii,5));  
     if tor(ii,3)<10^-3
         tor(ii,3)=0
     end
end

figure(1577)
plot(pos(:,7), tor(:,2))
grid on
title("Moment for theta2 motor")
xlabel("Time [sec]")
ylabel("Moment [Nm]")

figure(1578)
plot(pos(:,7), tor(:,3))
grid on
title("Moment for theta3 motor")
xlabel("Time [sec]")
ylabel("Moment [Nm]")

%4.1.10.2.3
%Position for the end effector X,Y,Z
figure(5)
plot(pos(:,7), matrix(:,1),"b.")
hold on 
plot(pos(:,7), matrix(:,2),"g.")
hold on
plot(pos(:,7), matrix(:,3),"r.")
legend("Position x","Position y","Position z")
title("Position (x,y,z) end effector")
xlabel("Time [sec]")
ylabel("Position [mm]")
grid on



%4.1.10
%stick simulation for the robotic arm
for k=1:length(pos)
    
    A1=[cos(pos(k,1)) -sin(pos(k,1)) 0 0 ; sin(pos(k,1)) cos(pos(k,1)) 0 0; 0 0 1 800; 0 0 0 1];
    A2=[1 0 0 0; 0 1 0 0; 0 0 1 (pos(k,2)); 0 0 0 1];
    A3=[1 0 0 0; 0 1 0 (pos(k,3)); 0 0 1 0; 0 0 0 1];   
    A4=[cos(pos(k,4)) 0 -sin(pos(k,4)) 0; 0 1 0 150; -sin(pos(k,4)) 0 cos(pos(k,4)) 0; 0 0 0 1]; 
    A5=[cos(pos(k,5)) -sin(pos(k,5))  0  -150*sin(pos(k,5)); sin(pos(k,5)) cos(pos(k,5)) 0 150*cos(pos(k,5)); 0 0 1 0; 0 0 0 1]; 
    A6=[cos(pos(k,6)) 0 sin(pos(k,6)) 0; 0 1 0 0; -sin(pos(k,6)) 1 cos(pos(k,6)) 0; 0 0 0 1];

    %location of every DOF
    d0_1=A1*[0;0;0;1];
    d0_2=A1*A2*[0;0;0;1];
    d0_3=A1*A2*A3*[0;0;0;1];
	d0_4=A1*A2*A3*A4*[0;0;0;1];
    d0_5=A1*A2*A3*A4*A5*[0;0;0;1];
    d0_6=A1*A2*A3*A4*A5*A6*[0;0;0;1];


    %position of theta1
    %base_link
    x1=[0,d0_1(1)];
    y1=[0,d0_1(2)];
    z1=[0,d0_1(3)];
    
    %position of l1    
    x2=[d0_1(1),d0_2(1)];
    y2=[d0_1(2),d0_2(2)];
    z2=[d0_1(3),d0_2(3)];
    
    %position of l2
    x3=[d0_2(1),d0_3(1)];
    y3=[d0_2(1),d0_3(2)];
    z3=[d0_2(3),d0_3(3)];
    
    %position of theta2
    x4=[d0_3(1),d0_4(1)];
    y4=[d0_3(2),d0_4(2)];
    z4=[d0_3(3),d0_4(3)];
    
   %position of theta3
    x5=[d0_4(1),d0_5(1)];
    y5=[d0_4(2),d0_5(2)];
    z5=[d0_4(3),d0_5(3)];
    
    %position of theta4
    %end effector
    x6=[d0_5(1),d0_6(1)];
    y6=[d0_5(2),d0_6(2)];
    z6=[d0_5(3),d0_6(3)];
    

    figure(2)
    %plot for the simulation
     plot3 (x1,y1,z1 , x2,y2,z2 , x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6)     
    grid on
axis([-1000 1000 -1000 1000 0 2000]) ;
xlabel('x');
ylabel('y');
zlabel('z');
x_pos(k)=d0_6(1)
y_pos(k)=d0_6(2)
z_pos(k)=d0_6(3)

%4.1.9.2
%3d x,y,z position

     %hold off
     
   
end


    figure(7)
     plot3 (x_pos ,y_pos,z_pos,"o",'LineWidth',1.5)
     hold on 
     plot3 (x_pos ,y_pos,z_pos,'LineWidth',1.5)

     xlabel('x','FontSize',15);
ylabel('y','FontSize',15);
zlabel('z','FontSize',15);
     hold on
    grid on
    

for k=1:length(pos)
     figure(10)
    %4.1.10.2.4
      plot (k, pos(k,2), 'm.')
      hold on
      plot (k, pos(k,3) , 'r.')
      hold on
      xlabel('time [sec]')
      ylabel('length [mm]')
      legend ('l1','l2')
      title("Length of l1,l2")
      grid on
      figure(11)
       plot (k, rad2deg(pos(k,1)), 'b.')
      hold on
      plot (k, rad2deg(pos(k,4)) , 'k*')
      hold on
      plot (k, rad2deg(pos(k,5)) , 'g+')
      hold on
      plot (k, rad2deg(pos(k,6)) , 'cV')
      hold on
      grid on
      legend ('theta1','theta2','theta3','theta4')
      xlabel('time [sec]')
      ylabel('angle [deg]')
      title("Deg of theta1 theta2 theta3 theta4")

end

