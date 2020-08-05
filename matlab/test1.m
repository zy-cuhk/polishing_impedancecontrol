clc,clear, close all
%% the robotic model
mdl_planar2;

%% the initial joints value
qz=[pi/4,-pi/2]';

% qua1=UnitQuaternion(rpy2r(rpy))
% qua2=UnitQuaternion(rpy2r(rpy_dsr))

rpy_dsr=[0,0,0];
Ko=-1;
Kp=1;
Ks=100;
delta_t=0.01;
fd=[-10,0,0];
I=eye(3);
S=[1 0 0;0 0 0;0 0 1];
xd=sqrt(2);
% for i=1:1:100
while(1)
    %% the computation of rotation matrix and rpy value
    T=p2.fkine(qz);
    Rot=zeros(3,3);
    Rot(1:3,1)=T.n;
    Rot(1:3,2)=T.o;
    Rot(1:3,3)=T.a;
    rpy=tr2rpy(Rot);
    
    %% the controller part 1: force control
    R_be=rpy2r([rpy(1),rpy(2),rpy(3)]);
    R_es=rpy2r([rpy(1),rpy(2),-rpy(3)]);
    x=T.t(1);
    if x>xd
        fnow_value=-Ks*(x-xd);
    else
        fnow_value=0;
    end
    theta=abs(rpy(3));
    fxnow_value=fnow_value*cos(theta);
    fynow_value=fnow_value*sin(theta);
    f=[fxnow_value,fynow_value,0];
    
    %% print the real-time force value
    str1=['the force value is : ',num2str(f)];
    disp(str1);
    
    v1=R_be*R_es*S*R_es'*Kp*(f-fd)';
    % v1=R_es*S*R_es'*Kp*(f-fd)'; which is experimentally ok
    v2=[0,0,0]';
    V(1:3,1)=v1+v2;
    
    %% the controller part 2: orientation control
    V(4:6,1)=Ko*(rpy-rpy_dsr);
    
    J=p2.jacob0(qz);
    qdot=pinv(J)*V;
    qz=qz+qdot*delta_t;
    
    %% print the output result
    str2=['the joints value are: ' num2str(qz(1)), ' and ',num2str(qz(2))];
    disp(str2);
    
    %% the visualization part
    p2.plot(qz')
    hold on;
    x=[xd,xd];
    y=[2,-2];
    plot(x,y,'LineWidth',5,'Color','k');
    view(0,90);
    hold off;
   
    %% the termination condition:
    error=rpy(3)-rpy_dsr(3);
    if abs(error)<0.01
        break;
    end
    
end




% trplot(Rot)
% mdl_ur5;
% rpy=tr2rpy(Rot);
% rpy2r(rpy);                                                                                          
% i=0;
% for i=1:1:1
%     q1=i/100*2*pi;
%     qz(1)=q1;
%     qz(3)=pi/2;
%     qz(4)=pi/2;
%     ur5.plot(qz);
%     hold on;
%     
%     T=ur5.fkine(qz);
%     Rot=zeros(3,3);
%     Rot(1:3,1)=T.n;
%     Rot(1:3,2)=T.o;
%     Rot(1:3,3)=T.a;
%     
%     % trplot(Rot)
%     drawnow
%     
% %     qi = ur5.ikine(T);
%     J = ur5.jacob0(qr);
%     det(J)
%     rank(J)
% end
%tranimate(Rot)








