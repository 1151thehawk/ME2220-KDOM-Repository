l1x = input('Enter the value of the crank l1x');
l1y = input('Enter the value of the crank l1y');
l2 = input('Enter the value of the connecting rod length l2');
l3 = input('Enter the value of the connecting rod length l3');
l4 = input('Enter the value of the connecting rod length l4');
theta3d = input('Give the initial guess for Theta 3(in deg)');
theta3=theta3d*pi/180;
theta4d = input('Give the initial guess for Theta 4(in deg)');
theta4=theta4d*pi/180;
theta2dotd=input('Enter the theta2dot');
theta2dot=theta2dotd*pi/30;
T3=[];
T4=[];
T3D=[];
T4D=[];
T3DD=[];
T4DD=[];
P=[];
Q=[];


for theta2=0:360
    
    dtheta3 = 0;
    dtheta4 = 0;
    flag=1;
    while(power(dtheta3,2)+power(dtheta4,2)>0.001|flag)
    
        J = [-l3*sin(theta3),-l4*sin(theta4);l3*cos(theta3),l4*cos(theta4)];
        F = [-l1x-l2*cos(theta2*pi/180)-l3*cos(theta3)-l4*cos(theta4);-l1y-l2*sin(theta2*pi/180)-l3*sin(theta3)-l4*sin(theta4)];
        X=inv(J)*F;
        dtheta3=X(1);
        dtheta4=X(2);
        Y=[theta3;theta4]+[dtheta3;dtheta4];
        theta3=Y(1);
        theta4=Y(2);
        flag=0;
    end
    FD = [theta2dot*l2*sin(theta2*pi/180);-l2*theta2dot*cos(theta2*pi/180)];
    Z=inv(J)*FD;
    theta3dot=Z(1);
    theta4dot=Z(2);
    
    FDD=[theta2dot*theta2dot*l2*cos(theta2*pi/180)+theta3dot*theta3dot*l3*cos(theta3)+theta4dot*theta4dot*l4*cos(theta4);theta2dot*theta2dot*l2*sin(theta2*pi/180)+theta3dot*theta3dot*l3*sin(theta3)+theta4dot*theta4dot*l4*sin(theta4)];
    K=inv(J)*FDD;
    theta3dotdot=K(1);
    theta4dotdot=K(2);
    
    za=(12-1.5*cot(theta2*pi/180))/(-((cot(theta4))*sin(theta2*pi/180))+cos(theta2*pi/180));
    p=-(1.5*cot(theta2*pi/180)+za*cos(theta2*pi/180))*0.01;
    q=-(1.5+za*sin(theta2*pi/180))*0.01;
    T3(end+1)=theta3*180/pi;
    T4(end+1)=theta4*180/pi;
    T3D(end+1)=theta3dot*60/(2*pi);
    T4D(end+1)=theta4dot*60/(2*pi);
    T3DD(end+1)=theta3dotdot;
    T4DD(end+1)=theta4dotdot;
    P(end+1)=p;
    Q(end+1)=q;
end


theta2=0:360;
figure(1)
plot(theta2,T4);
xlabel('Theta 2');
ylabel('Theta 4');

figure(2)
plot(theta2,T4D);
xlabel('Theta 2');
ylabel('w4');

figure(3)
plot(theta2,T4DD);
xlabel('Theta 2');
ylabel('alpha4');

figure(4)
plot(theta2,P);
xlabel('Theta 2');
ylabel('X-coordinate of P13 in m');

figure(5)
plot(theta2,Q);
xlabel('Theta 2');
ylabel('Y-coordinate of P13 in m');
