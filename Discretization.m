%Discretization (4.1.5)
%the running takes a lot of time, we do not recommend running the loop.

N=5;

%Variable
x1 = linspace(-pi,pi,N);
x2 = linspace(-pi,pi,N);
x3 = linspace(-3*pi/4,3*pi/4,N);
x4 = linspace(-pi,pi,N);
l1 = linspace(0,500,N);
l2 = linspace(0,500,N);

%loop
i=1;
j=1;
k=1;
l=1;
m=1;

%Isometric
for i=1:N
    x=- 150*sin(x1(i)) - 150*cos(x3(k))*sin(x1(i)) - l2(l)*sin(x1(i)) - 150*cos(x1(i))*cos(x2(m))*sin(x3(k));
    y= 150*cos(x1(i)) + 150*cos(x1(i))*cos(x3(k)) + l2(l)*cos(x1(i)) - 150*cos(x2(m))*sin(x1(i))*sin(x3(k));
    z= l1(j) + 150*sin(x2(m))*sin(x3(k)) + 800;
    plot3(x,y,z,'b.');
    grid on
    hold on
    for j=1:N
       x=- 150*sin(x1(i)) - 150*cos(x3(k))*sin(x1(i)) - l2(l)*sin(x1(i)) - 150*cos(x1(i))*cos(x2(m))*sin(x3(k));
       y= 150*cos(x1(i)) + 150*cos(x1(i))*cos(x3(k)) + l2(l)*cos(x1(i)) - 150*cos(x2(m))*sin(x1(i))*sin(x3(k));
       z= l1(j) + 150*sin(x2(m))*sin(x3(k)) + 800;
     plot3(x,y,z,'b.');
        hold on
        for k=1:N 
         x=-150*sin(x1(i)) - 150*cos(x3(k))*sin(x1(i)) - l2(l)*sin(x1(i)) - 150*cos(x1(i))*cos(x2(m))*sin(x3(k));
         y=150*cos(x1(i)) + 150*cos(x1(i))*cos(x3(k)) + l2(l)*cos(x1(i)) - 150*cos(x2(m))*sin(x1(i))*sin(x3(k));
         z= l1(j) + 150*sin(x2(m))*sin(x3(k)) + 800;
     plot3(x,y,z,'b.');
            hold on
            for l=1:N    
            x= -150*sin(x1(i)) -150*cos(x3(k))*sin(x1(i)) - l2(l)*sin(x1(i)) - 150*cos(x1(i))*cos(x2(m))*sin(x3(k));
            y= 150*cos(x1(i)) + 150*cos(x1(i))*cos(x3(k)) + l2(l)*cos(x1(i)) - 150*cos(x2(m))*sin(x1(i))*sin(x3(k));
            z= l1(j) + 150*sin(x2(m))*sin(x3(k)) + 800;
            plot3(x,y,z,'b.');
                hold on
                for m=1:N 
                x=- 150*sin(x1(i)) - 150*cos(x3(k))*sin(x1(i)) - l2(l)*sin(x1(i)) - 150*cos(x1(i))*cos(x2(m))*sin(x3(k));
                y= 150*cos(x1(i)) + 150*cos(x1(i))*cos(x3(k)) + l2(l)*cos(x1(i)) - 150*cos(x2(m))*sin(x1(i))*sin(x3(k));
                z= l1(j) + 150*sin(x2(m))*sin(x3(k)) + 800;
                plot3(x,y,z,'b.');
                    hold on
                    xlabel('X');
                   ylabel('Y');
                   zlabel('Z');
                end
            end
        end 
   end
end        
