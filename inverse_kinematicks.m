%Inverse kinematics (4.1.6)



A1=[cos(theta1) -sin(theta1) 0 0 ; sin(theta1) cos(theta1) 0 0 ;0 0 1 800 ; 0 0 0 1];
A2=[1 0 0 0;0 1 0 0; 0 0 1 l1; 0 0 0 1];
A3=[1 0 0 0;0 1 0 l2; 0 0 1 0; 0 0 0 1];
A4=[cos(theta2) 0 sin(theta2) 0; 0 1 0 150; -sin(theta2) 0 cos(theta2) 0; 0 0 0 1];
A5=[cos(theta3) -sin(theta3) 0 -150*sin(theta3) ; sin(theta3) cos(theta3) 0 150*cos(theta3) ; 0 0 1 0 ; 0 0 0 1 ];
A6=[cos(theta4) 0 sin(theta4) 0 ; 0 1 0 0 ; -sin(theta4) 0 cos(theta4) 0; 0 0 0 1];

R1=A1(1:3,1:3);
R2=A2(1:3,1:3);
R3=A3(1:3,1:3);
R4=A4(1:3,1:3);
R5=A5(1:3,1:3);
R6=A6(1:3,1:3);

 
%The user enters coordinates (forword
%kinematics)
x=An(1,4)
y=An(2,4)
z=An(3,4)

%orientation
Rn=An(1:3,1:3);

%First 3 degrees of freedom     

%L1
l1=z-800-150*Rn(3,2)
C=sqrt((x-150*Rn(1,2))^2+(y-150*Rn(2,2))^2)

%L2
l2=C-150

%T1
theta1=atan2(-1*(x-150*Rn(1,2))/(l2+150),(y-150*Rn(2,2))/(l2+150))
T1=theta1*(180/pi);
c=sqrt(x^2+y^2);

%First 3 degrees of freedom
    

R0_3=R1*R2*R3
Rt=transpose(R0_3)*Rn;
R3_6=R4*R5*R6;

%T3
theta3=atan2(sqrt(Rt(2,1)^2+Rt(2,3)^2),Rt(2,2)); %Top or bottom arm
T3=theta3*180/pi

%T2
theta2=atan2(Rt(3,2)/sin(theta3),-Rt(1,2)/sin(theta3));
T2=theta2*180/pi

%T4
theta4=atan2(Rt(2,3)/sin(theta3), Rt(2,1)/sin(theta3));
T4=theta4*180/pi



