%Jacobian and statics (4.1.7, 4.1.8)

%Variable and constant
syms theta1 theta2 theta3 theta4 ; 
syms l1 l2 ;

    %transformation matrix   
A1=[cos(theta1) -sin(theta1) 0 0 ; sin(theta1) cos(theta1) 0 0; 0 0 1 800; 0 0 0 1];
A2=[1 0 0 0; 0 1 0 1; 0 0 1 l1; 0 0 0 1];
A3=[1 0 0 0; 0 1 0 l2; 0 0 1 0; 0 0 0 1];   
A4=[cos(theta2) 0 -sin(theta2) 0; 0 1 0 150; -sin(theta2) 0 cos(theta2) 0; 0 0 0 1]; 
A5=[cos(theta3) -sin(theta3)  0  -150*sin(theta3); sin(theta3) cos(theta3) 0 150*cos(theta3); 0 0 1 0; 0 0 0 1]; 
A6=[cos(theta4) 0 sin(theta4) 0; 0 1 0 0; -sin(theta4) 1 cos(theta4) 0; 0 0 0 1];   
    An=A1*A2*A3*A4*A5*A6;
    
    %transformation rotation matrix 
    R1=A1(1:3,1:3)
    R2=A2(1:3,1:3)
    R3=A3(1:3,1:3);
    R4=A4(1:3,1:3);
    R5=A5(1:3,1:3);
    R6=A6(1:3,1:3);
    Rn=simplify(R1*R2*R3*R4*R5*R6);
    
    %Location vector
    d0_1=(A1*[0;0;0;1]);
    d0_2=(A1*A2*[0;0;0;1]);
    d0_3=(A1*A2*A3*[0;0;0;1]);
    d0_4=(A1*A2*A3*A4*[0;0;0;1]);
    d0_5=(A1*A2*A3*A4*A5*[0;0;0;1]);
    d0_6=(A1*A2*A3*A4*A5*A6*[0;0;0;1]);
    dn=d0_6(1:3)
    
%Jacobian (4.1.7)

    %theta1 Jacobian (Rotary joint) (First degree of freedom) 
    b0=[0;0;1]; 
    r0e=dn; 
    br0=simplify(cross(b0,r0e)); 
    c0=diff(dn,theta1) %Numerical test
    br0-c0; %If the result is zero it's true
   
    %l2 Jacobian (Linear joint)
    b1=(R1*[0;0;1]);
    c1=diff(dn,l1);
    b1-c1;
    
    %constant joint  
    b2=(R1*R2*[0;0;1]);
    r2e=(dn-d0_2(1:3));
    br2=simplify(cross(b2,r2e));
    c2=diff(dn,l2);
    br2-c2;
    
    %theta2 Jacobian (Rotary joint)
    b3=(R1*R2*R3*[0;0;1]);
    r3e=(dn-d0_3(1:3));
    br3=simplify(cross(b3,r3e))
    c3=simplify(diff(dn,theta2))
    simplify(br3-c3)
    
    %theta3 Jacobian (Rotary joint)
    b4=(R1*R2*R3*R4*[0;0;1]);
    r4e=(dn-d0_4(1:3));
    br4=simplify(cross(b4,r4e));
    c4=simplify(diff(dn,theta3))
    simplify(br4-c4);
    
    %theta4 Jacobian (Rotary joint)
    b5=(R1*R2*R3*R4*R5*[0;0;1]);
    r5e=(dn-d0_5(1:3));
    br5=simplify(cross(b5,r5e));
    c5=simplify(diff(dn,theta4));
    simplify(br5-c5);
    
    % Calculate the forward kinematics
T = simplify(A1*A2*A3*A4*A5*A6)
R=T(1:3,1:3)


% Extract end effector position
p = T(1:3, 4);

% Calculate the Jacobian matrix
JL = simplify([diff(p, theta1) diff(p, l1) diff(p, l2) diff(p, theta2) diff(p, theta3) diff(p, theta4)])
    %Rotary Jacobian 
    %JL=[br0, b1, b2, br3, br4, br5];
    %Linear Jacobiaמ
    JA=[b0,[0;0;0],[0;0;0],b3,b4,b5];
    %The Jacobiaמ
    J=simplify([JL;JA])

%statics (4.1.8)

    %const
    syms Mg %the force
    F=[0;0;-Mg]; %forces
    T=[0;0;0]; %Torques
    tau=-simplify(transpose(J)*[F;T]) %Vector forces for each joint

