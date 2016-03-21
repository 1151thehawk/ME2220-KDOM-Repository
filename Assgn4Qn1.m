theta3d = input('Give the initial guess for Theta 3(in deg)');
theta3=theta3d*pi/180;
theta4d = input('Give the initial guess for Theta 4(in deg)');
theta4=theta4d*pi/180;
betad=input('Enter the angular velocity of crank');
betadot=betad;
k4=45*0.001;
m4=17;
T3=[];
T4=[];
T3D=[];
T4D=[];
T3DD=[];
T4DD=[];
TQ4=[];
PBX=[];
PBY=[];
PCX=[];
PCY=[];
MDRIVE=[];

for beta=0:360
    
    dtheta3 = 0;
    dtheta4 = 0;
    flag=1;
    theta2=360-beta;
    while(power(dtheta3,2)+power(dtheta4,2)>0.001|flag)
    
        J = [-110*sin(theta3),-40*sin(theta4);110*cos(theta3),40*cos(theta4)];
        F = [120-25*cos(beta*pi/180)-110*cos(theta3)-40*cos(theta4);25*sin(beta*pi/180)-110*sin(theta3)-40*sin(theta4)];
        X=inv(J)*F;
        dtheta3=X(1);
        dtheta4=X(2);
        Y=[theta3;theta4]+[dtheta3;dtheta4];
        theta3=Y(1);
        theta4=Y(2);
        flag=0;
    end
    FD = [betadot*25*sin(beta*pi/180);25*betadot*cos(beta*pi/180)];
    Z=inv(J)*FD;
    theta3dot=Z(1);
    theta4dot=Z(2);
    
    FDD=[betadot*betadot*25*cos(beta*pi/180)+theta3dot*theta3dot*110*cos(theta3)+theta4dot*theta4dot*40*cos(theta4);-betadot*betadot*25*sin(beta*pi/180)+theta3dot*theta3dot*110*sin(theta3)+theta4dot*theta4dot*40*sin(theta4)];
    K=inv(J)*FDD;
    theta3dotdot=K(1);
    theta4dotdot=K(2);
    
    torque4=m4*k4*k4*theta4dotdot
    
    Fprime=[0;0;0;0;0;0;0;-m4*9.8;0.5];
    
    H1=[0;0;0;0;0;0;(-m4*0.025*cos(theta2*pi/180)*betadot*betadot)+(m4*0.11*cos(theta3)*theta3dot*theta3dot)+(m4*0.015*cos(theta4)*theta4dot*theta4dot);(-m4*0.025*sin(theta2*pi/180)*betadot*betadot)+(m4*0.11*sin(theta3)*theta3dot*theta3dot)+(m4*0.015*sin(theta4)*theta4dot*theta4dot);0];
    H2=[0,0,0;0,0,0;0,0,0;0,0,0;0,0,0;0,0,0;-m4*0.025*sin(theta2*pi/180),m4*0.11*sin(theta3),m4*0.015*sin(theta4);m4*0.025*cos(theta2*pi/180),-m4*0.11*cos(theta3),-m4*0.015*cos(theta4);0,0,m4*k4*k4]*[0;theta3dotdot;theta4dotdot];
    
    H=H1+H2;
    
    Qprime=[1,0,1,0,0,0,0,0,0;0,1,0,1,0,0,0,0,0;0,0,-0.025*sin(theta2*pi/180),0.025*cos(theta2*pi/180),0,0,0,0,1;0,0,-1,0,1,0,0,0,0;0,0,0,-1,0,1,0,0,0;0,0,0.11*sin(theta3),-0.11*cos(theta3),0,0,0,0,0;0,0,0,0,-1,0,1,0,0;0,0,0,0,0,-1,0,1,0;0,0,0,0,0.015*sin(theta4),-0.015*cos(theta4),0.025*sin(theta4),-0.025*cos(theta4),0]
    
    Pprime=inv(Qprime)*(H-Fprime);
    
    T3(end+1)=theta3*180/pi;
    T4(end+1)=theta4*180/pi;
    T3D(end+1)=theta3dot;
    T4D(end+1)=theta4dot;
    T3DD(end+1)=theta3dotdot;
    T4DD(end+1)=theta4dotdot;
    TQ4(end+1)=torque4;
    PBX(end+1)= Pprime(3);
    PBY(end+1)= Pprime(4);
    PCX(end+1)= Pprime(5);
    PCY(end+1)= Pprime(6);
    MDRIVE(end+1)= Pprime(9);
    
end


beta=0:360;

figure(1)
plot(beta,T4DD);
xlabel('Beta');
ylabel('alpha4');

figure(2)
plot(beta,TQ4);
xlabel('Beta');
ylabel('Torque on link 4');

figure(3)
plot(beta,PBX);
xlabel('Beta');
ylabel('X Force on joint B');

figure(4)
plot(beta,PBY);
xlabel('Beta');
ylabel('Y Force on joint B');

figure(5)
plot(beta,PCX);
xlabel('Beta');
ylabel('X Force on joint C');

figure(6)
plot(beta,PCY);
xlabel('Beta');
ylabel('Y Force on joint C');

figure(7)
plot(beta,MDRIVE);
xlabel('Beta');
ylabel('Driving Torque');
