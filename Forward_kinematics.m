
%Variable
syms theta1 theta2 theta3 theta4 ; 
syms l1 l2;


%Forward kinematics(4.1.4)
    %transformation matrix (4.1.4.1)
    A1=[cos(theta1) -sin(theta1) 0 0 ; sin(theta1) cos(theta1) 0 0; 0 0 1 800; 0 0 0 1];
    A2=[1 0 0 0; 0 1 0 0; 0 0 1 l1; 0 0 0 1];
    A3=[1 0 0 0; 0 1 0 l2; 0 0 1 0; 0 0 0 1];   
    A4=[cos(theta2) 0 -sin(theta2) 0; 0 1 0 150; -sin(theta2) 0 cos(theta2) 0; 0 0 0 1]; 
    A5=[cos(theta3) -sin(theta3)  0  -150*sin(theta3); sin(theta3) cos(theta3) 0 150*cos(theta3); 0 0 1 0; 0 0 0 1]; 
    A6=[cos(theta4) 0 sin(theta4) 0; 0 1 0 0; -sin(theta4) 1 cos(theta4) 0; 0 0 0 1];
    
   
    %transformation matrix end unit(4.1.4.2)
    An=A1*A2*A3*A4*A5*A6
   
    %check point (4.1.4.3)
    pointheta1=[0, 0, pi/2, 0, 250,0];
    pointheta2=[0, pi/2, pi/2, 0, 250, 250];
    pointheta3=[pi, 0, 0, 0, 500, 500];
    pointheta4=[pi, pi/2, pi/2, pi, 500, 500];
    point5=[pi, 1.5*pi, 0, pi, 0, 0];
    point=[pointheta1; pointheta2; pointheta3; pointheta4; point5]; %Define a matrix for the points

for (k=1:5)
    pp=point(k,:); %point 1-5
    A=subs(An, {theta1, theta2, theta3, theta4, l1, l2}, pp); %The transformation matrix for each point
   
    %End unit location
    d0_1=subs((A1*[0;0;0;1]), {theta1, theta2, theta3, theta4, l1, l2}, {pp});
    d0_2=subs((A1*A2*[0;0;0;1]), {theta1, theta2, theta3, theta4, l1, l2}, {pp});
    d0_3=subs((A1*A2*A3*[0;0;0;1]), {theta1, theta2, theta3, theta4, l1, l2}, {pp});
    d0_4=subs((A1*A2*A3*A4*[0;0;0;1]), {theta1, theta2, theta3, theta4, l1, l2}, {pp});
    d0_5=subs((A1*A2*A3*A4*A5*[0;0;0;1]), {theta1, theta2, theta3, theta4, l1, l2}, {pp});
    d0_6=subs((A1*A2*A3*A4*A5*A6*[0;0;0;1]), {theta1, theta2, theta3, theta4, l1, l2}, {pp});
    
    %Location of each joint
    x1=[0,0];
    y1=[0,0];
    z1=[0,d0_1(3)];

    x2=[0,d0_2(1)];
    y2=[0,d0_2(2)];
    z2=[d0_1(3),d0_2(3)];

    x3=[d0_2(1),d0_3(1)];
    y3=[d0_2(2),d0_3(2)];
    z3=[d0_2(3),d0_3(3)];
    
    x4=[d0_3(1),d0_4(1)];
    y4=[d0_3(2),d0_4(2)];
    z4=[d0_3(3),d0_4(3)];
   
    x5=[d0_4(1),d0_5(1)];
    y5=[d0_4(2),d0_5(2)];
    z5=[d0_4(3),d0_5(3)];
    
    x6=[d0_5(1),d0_6(1)];
    y6=[d0_5(2),d0_6(2)];
    z6=[d0_5(3),d0_6(3)];
   
    %plots
    subplot(2,3,k)
    plot3(x1,y1,z1 , x2,y2,z2 , x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6)
    hold on
    title ('point',k)
    axis([-1000 1000 -1000 1000 0 2000])
    grid
end
    
