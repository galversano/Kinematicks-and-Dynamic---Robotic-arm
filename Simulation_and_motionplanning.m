%The simulation (without graphs)

%const

%Initial and final location
ds=input('enter start piont, like [X;Y;Z]:')
dg=input('enter end piont, like [X;Y;Z]:')

%Roll pitch yaw (start)
 roll=input('enter start roll:') ;
 pitch=input('enter start pitch:'); 
 yaw=input('enter start yaw:'); 
 
 
 nx=cos(tx)*sin(ty);
 ny=sin(tx)*sin(ty);
 nz=cos(ty);
 k=1-cos(tz);
 
 
R_x=[1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
R_y=[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
R_z=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];

Rs= R_z*R_y*R_x;

 
 Rs=[cos(tz)+k*nx^2,nx*ny*k-nz*sin(tz),nx*nz*k+ny*sin(tz);
 nx*ny*k+nz*sin(tz),k*ny^2+cos(tz),nz*ny*k-nx*sin(tz);
 nx*nz*k-ny*sin(tz),nz*ny*k+nx*sin(tz),k*nz^2+cos(tz)]
 
Rs= R_z*R_y*R_x;

%Roll pitch yaw (end)
 roll=input('enter end roll:') 
 pitch=input('enter end pitch:') 
 yaw=input('enter end yaw:') 
 
 nx=cos(tx)*sin(ty);
 ny=sin(tx)*sin(ty);
 nz=cos(ty);
 k=1-cos(tz);
 
 Rg=[cos(tz)+k*nx^2,nx*ny*k-nz*sin(tz),nx*nz*k+ny*sin(tz);
 nx*ny*k+nz*sin(tz),k*ny^2+cos(tz),nz*ny*k-nx*sin(tz);
 nx*nz*k-ny*sin(tz),nz*ny*k+nx*sin(tz),k*nz^2+cos(tz)]

R_x=[1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
R_y=[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
R_z=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];

Rg= R_z*R_y*R_x;

Rb=Rg*transpose(Rs);

%angle
tetal=acos((trace(Rb)-1)/2);

%eigenvector
if tetal==0
    n=[0;0;0];
else
    n(1)=(Rb(3,2)-Rb(2,3))/(2*sin(tetal));
    n(2)=(Rb(1,3)-Rb(3,1))/(2*sin(tetal));
    n(3)=(Rb(2,1)-Rb(1,2))/(2*sin(tetal));
end

%now we have n and teta
%time
T=30;
t_1=T/3;
t_2=(2*T)/3;
a=1/(0.5*t_1^2+t_1*(t_2-t_1)+0.5*(2*t_1-(T-t_2))*(T-t_2));

%location
arry_point=ds;
arry_matrix=Rs;
for t=1:T
if 0<t && t<t_1
s=0.5*a*t^2;
else
    if t_1<=t && t<=t_2
    s=0.5*a*t_1^2+a*t_1*(t-t_1);
    
    else
        if t_2<t && t<=T
    s=0.5*a*t_1^2+a*t_1*(t_2-t_1)+0.5*(t-t_2)*(2*a*t_1-a*(t-t_2));

        end
    end
end
d_t=ds+s*(dg-ds);
arry_point(1,t+1)=d_t(1);
arry_point(2,t+1)=d_t(2);
arry_point(3,t+1)=d_t(3);

%Orientation
teta=tetal*s
k=1-cos(teta);
Rot=[cos(teta)+k*n(1)^2,n(1)*n(2)*k-n(3)*sin(teta),n(1)*n(3)*k+n(2)*sin(teta);
n(1)*n(2)*k+n(3)*sin(teta),k*n(2)^2+cos(teta),n(3)*n(2)*k-n(1)*sin(teta);
n(1)*n(3)*k-n(2)*sin(teta),n(3)*n(2)*k+n(1)*sin(teta),k*n(3)^2+cos(teta)]
Rt=Rot*Rs;
arry_matrix(:,:,t+1)=Rt;

end

%grid on
%now we have matrix with linear end pionts


figure(1)
%plot(1:29,v(1:29))
ylabel("Velocity [m/s]")
xlabel("Time [sec]")
grid on
figure (3)

plot3(arry_point(1,2:30),arry_point(2,2:30),arry_point(3,2:30), 'r.')
xlabel('x')
ylabel('y')
zlabel('z')
grid on


j=1;
for j=1:31

    x=arry_point(1,j);
    y=arry_point(2,j);
    z=arry_point(3,j);
    
    Rn=arry_matrix(:,:,j)
   
    Pc=[x;y;z];
    x=Pc(1);
    y=Pc(2);
    z=Pc(3);
         figure(4)
    %4.1.10.2.3
     plot (j, x, 'b.')
     hold on
     plot (j, y, 'm.')
     hold on
     plot (j, z , 'r.')
     hold on
     xlabel('time [sec]'),ylabel('position [m]')
     legend ('x','y','z')
     grid on
    
    

    %Inverse_kinematics
    l1=z-800-150*Rn(3,2)
    %if l1<0
     %   l1=0
    %end
    C=sqrt((x-150*Rn(1,2))^2+(y-150*Rn(2,2))^2)
    l2=(C-150)
   % if l2>500
    %    l2=500
    %end
    %if l2<0
     %   l2=0;
    %end

    theta1=atan2(-1*(x-150*Rn(1,2))/(l2+150),(y-150*Rn(2,2))/(l2+150))
    
    A1=[cos(theta1) -sin(theta1) 0 0 ; sin(theta1) cos(theta1) 0 0 ;0 0 1 800 ; 0 0 0 1];
    A2=[1 0 0 0;0 1 0 0; 0 0 1 l1; 0 0 0 1];
    A3=[1 0 0 0;0 1 0 l2; 0 0 1 0; 0 0 0 1];

    R1=A1(1:3,1:3);
    R2=A2(1:3,1:3);
    R3=A3(1:3,1:3);
    %R4=A4(1:3,1:3);

R0_3=R1*R2*R3;
Rtr=transpose(R0_3)*Rn;

theta3=atan2(sqrt(Rtr(2,1)^2+Rtr(2,3)^2),Rtr(2,2)); %Top or bottom arm
if theta3>(3*pi)/4 | theta3<-(3*pi)/4
    theta3=(3*pi)/4
end
if theta3==0
    theta3=atan2(-Rtr(2,2),sqrt(Rtr(2,1)^2+Rtr(2,3)^2)); 
end%Top or bottom arm
    

T3=theta3*180/pi
 % (0<theta3<180)
%T2
theta2=atan2(Rtr(3,2)/sin(theta3),-Rtr(1,2)/sin(theta3));
    
    
T2=theta2*180/pi
%theta2_1=atan2(Rt(1,2)/sin(theta3),Rt(3,2)/sin(theta3));

%T4
theta4=atan2(Rtr(2,3)/sin(theta3), Rtr(2,1)/sin(theta3));
T4=theta4*180/pi

A4=[cos(theta2) 0 sin(theta2) 0; 0 1 0 0; -sin(theta2) 0 cos(theta2) 0; 0 0 0 1];
A5=[1 0 0 0; 0 1 0 150; 0 0 1 0; 0 0 0 1];

A6=[cos(theta3) -sin(theta3) 0 0 ; sin(theta3) cos(theta3) 0 0 ; 0 0 1 0 ; 0 0 0 1 ];
A7=[1 0 0 0 ; 0 1 0 150 ; 0 0 1 0; 0 0 0 1];
A8=[cos(theta4) 0 sin(theta4) 0 ; 0 1 0 0 ; -sin(theta4) 0 cos(theta4) 0; 0 0 0 1];

    %location of every DOF
    d0_1=A1*[0;0;0;1];
    d0_2=A1*A2*[0;0;0;1];
    d0_3=A1*A2*A3*[0;0;0;1];
	d0_4=A1*A2*A3*A4*[0;0;0;1];
    d0_5=A1*A2*A3*A4*A5*[0;0;0;1];
    d0_6=A1*A2*A3*A4*A5*A6*[0;0;0;1];
    d0_7=A1*A2*A3*A4*A5*A6*A7*[0;0;0;1];
    d0_8=A1*A2*A3*A4*A5*A6*A7*A8*[0;0;0;1];


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
    
    x7=[d0_6(1),d0_7(1)];
    y7=[d0_6(2),d0_7(2)];
    z7=[d0_6(3),d0_7(3)];
    
    x8=[d0_7(1),d0_8(1)];
    y8=[d0_7(2),d0_8(2)];
    z8=[d0_7(3),d0_8(3)];

    figure(2)
     plot3 (x1,y1,z1 , x2,y2,z2 , x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7 ,x8,y8,z8)     
    grid on
axis([-1000 1000 -1000 1000 0 2000]) ;
xlabel('x');
ylabel('y');
zlabel('z');


     %hold off
  figure(5)
    %4.1.10.2.4
      plot (j, theta1, 'b.')
      hold on
      plot (j, l1, 'm.')
      hold on
      plot (j, l2 , 'r.')
      hold on
      plot (j, theta2 , 'k*')
      hold on
      plot (j, theta3 , 'g+')
      hold on
      plot (j, theta4 , 'cV')
      hold on
      xlabel('time [sec]')
      ylabel('angel [rad] or length [m]')
      legend ('theta1','l1','l2','theta2','theta3','theta4')
      grid on
    
 %4.10....
 
 
figure(3)
     Mg=5;  
     tau=[0;Mg;0;150*Mg*cos(theta2)*sin(theta3);150*Mg*cos(theta3)*sin(theta2) ;0]
     plot (j, tau(1), 'b.')
     hold on
     plot (j, tau(2), 'm.')
     hold on
     plot (j, tau(3) , 'r.')
     hold on
     plot (j, tau(4) , 'k*')
     hold on
     plot (j, tau(5) , 'g+')
     hold on
     plot (j, tau(6) , 'cV')
     hold on
     xlabel('time [sec]')
     ylabel('Torues [Nm]')
     legend ('theta1','l1','l2','theta3','theta4','theta5')
     grid on
   %}
  %  pause(0.3)
     
    pause(0.3)
end
