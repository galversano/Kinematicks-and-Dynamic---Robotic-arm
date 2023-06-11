%Initial and final location
ds=input('enter start piont, like [X;Y;Z]:')
dg=input('enter end piont, like [X;Y;Z]:')
%Rs=input('enter start orientetion, (matrix 3*3):')
%Rg=input('enter end orientetion, (matrix 3*3):')

%Roll pitch yaw (start)
 tx=input('enter start roll:') 
 ty=input('enter start pitch:'); 
 tz=input('enter start yaw:') 
 
 nx=cos(tx)*sin(ty);
 ny=sin(tx)*sin(ty);
 nz=cos(ty);
 k=1-cos(tz);
 
 Rs=[cos(tz)+k*nx^2,nx*ny*k-nz*sin(tz),nx*nz*k+ny*sin(tz);
 nx*ny*k+nz*sin(tz),k*ny^2+cos(tz),nz*ny*k-nx*sin(tz);
 nx*nz*k-ny*sin(tz),nz*ny*k+nx*sin(tz),k*nz^2+cos(tz)]
 

%Roll pitch yaw (end)
 tx=input('enter end roll:') 
 ty=input('enter end pitch:') 
 tz=input('enter end yaw:') 
 
 nx=cos(tx)*sin(ty);
 ny=sin(tx)*sin(ty);
 nz=cos(ty);
 k=1-cos(tz);
 
 Rg=[cos(tz)+k*nx^2,nx*ny*k-nz*sin(tz),nx*nz*k+ny*sin(tz);
 nx*ny*k+nz*sin(tz),k*ny^2+cos(tz),nz*ny*k-nx*sin(tz);
 nx*nz*k-ny*sin(tz),nz*ny*k+nx*sin(tz),k*nz^2+cos(tz)]

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
%location
array_point(1,1)=ds(1)
array_point(2,1)=ds(2)
array_point(3,1)=ds(3)
arry_matrix=Rs;
velos(1)=0;
for t=1:T
if 0<t && t<t_1
s=0.5*a*t^2;
s_dot=a*t;
else
    if t_1<=t && t<=t_2
    s=0.5*a*t_1^2+a*t_1*(t-t_1);
    s_dot=a*t_1;
    
    else
        if t_2<t && t<=T
    s=0.5*a*t_1^2+a*t_1*(t_2-t_1)+0.5*(t-t_2)*(2*a*t_1-a*(t-t_2));
    s_dot=a*t_1-a*(t-t_2);

        end
    end
end
velos(t+1)=s_dot;
d_t=ds+s*(dg-ds);
v_t=s_dot*(dg-ds);
arry_point(1,t+1)=d_t(1);
arry_point(2,t+1)=d_t(2);
arry_point(3,t+1)=d_t(3);

arry_point_v(1,t+1)=v_t(1);
arry_point_v(2,t+1)=v_t(2);
arry_point_v(3,t+1)=v_t(3);

%Orientation
teta=tetal*s
k=1-cos(teta);
Rot=[cos(teta)+k*n(1)^2,n(1)*n(2)*k-n(3)*sin(teta),n(1)*n(3)*k+n(2)*sin(teta);
n(1)*n(2)*k+n(3)*sin(teta),k*n(2)^2+cos(teta),n(3)*n(2)*k-n(1)*sin(teta);
n(1)*n(3)*k-n(2)*sin(teta),n(3)*n(2)*k+n(1)*sin(teta),k*n(3)^2+cos(teta)]
Rt=Rot*Rs;
arry_matrix(:,:,t+1)=Rt;

end

%trapzoid motion
figure(98)
plot(0:30,velos,'LineWidth',1.5)
ylabel("Velocity [mm/s]",'FontSize',14)
xlabel("Time [sec]",'FontSize',14)
%xlim([0 28]);
ylim([0 0.06]);
grid on

%velocity end effector x,y,z
figure(99)
subplot(3, 1, 1);
plot(0:30,arry_point_v(1,:),'LineWidth',1.5)
hold on
plot(0:30,arry_point_v(1,:),'o','LineWidth',1.5)
ylabel("Velocity (x) [mm/s]",'FontSize',14)
xlabel("Time [sec]",'FontSize',14)
grid minor
subplot(3, 1, 2);
plot(0:30,arry_point_v(2,:),'LineWidth',1.5)
hold on
plot(0:30,arry_point_v(2,:),'o','LineWidth',1.5)
ylabel("Velocity (y) [mm/s]",'FontSize',14)
xlabel("Time [sec]",'FontSize',14)
%xlim([0 28]);
grid minor
subplot(3, 1, 3);
plot(0:30,arry_point_v(3,:),'LineWidth',1.5)
hold on
plot(0:30,arry_point_v(3,:),'o','LineWidth',1.5)
ylabel("Velocity (z) [mm/s]",'FontSize',14)
xlabel("Time [sec]",'FontSize',14)
%xlim([0 28]);
grid minor

%Position end effector x,y,z
arry_point(1,1)=ds(1)
arry_point(2,1)=ds(2)
arry_point(3,1)=ds(3)

figure(100)
subplot(3, 1, 1);
plot(0:30,arry_point(1,:),'LineWidth',1.5)
hold on
plot(0:30,arry_point(1,:),'o','LineWidth',1.5)
ylabel("Position (x) [mm]",'FontSize',14)
xlabel("Time [sec]",'FontSize',14)
grid minor
subplot(3, 1, 2);
plot(0:30,arry_point(2,:),'LineWidth',1.5)
hold on
plot(0:30,arry_point(2,:),'o','LineWidth',1.5)
ylabel("Position (y) [mm]",'FontSize',14)
xlabel("Time [sec]",'FontSize',14)
%xlim([0 28]);
grid minor
subplot(3, 1, 3);
plot(0:30,arry_point(3,:),'LineWidth',1.5)
hold on
plot(0:30,arry_point(3,:),'o','LineWidth',1.5)
ylabel("Position (z) [mm]",'FontSize',14)
xlabel("Time [sec]",'FontSize',14)
%xlim([0 28]);
grid minor


%grid on
%now we have matrix with linear end pionts

figure (3)
plot3(arry_point(1,2:30),arry_point(2,2:30),arry_point(3,2:30),  'o','LineWidth',1.2)
hold on
plot3(arry_point(1,2:30),arry_point(2,2:30),arry_point(3,2:30),'LineWidth',1.5)
xlim([-0.4 0.4])
xlabel('x','FontSize',14)
ylabel('y','FontSize',14)
zlabel('z','FontSize',14)
grid on


