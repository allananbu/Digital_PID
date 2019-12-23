clc
clear all
close all

num = [14.3];
den = [0.00333 0.201 1];

sys = tf(num,den);

sys_d = c2d(sys,0.1);

%PID controller parameters
kp = 1;
kd = 0.5;
ki = 6;
ts=0.1;
ti=(ts/ki);
td=(kd*ts);

rr1 = [];
c =[]; e = []; u1=[];y1=[];
u1(1)=0;u1(2)=0;u1(3)=0; %previous control input
e(1) = 0; e(2) =0; e(3) =0; %previous error
y1(1)=0;y1(2)=0;
r1(1)=0;r1(2)=0;
err=0;
duu=0;
spp=[];
tic

for k1=3:101
      
    if ((k1>=3)&&(k1<=40))
        r1(k1)=4;
    elseif ((k1>=41)&&(k1<=102))
        r1(k1)=6;
        %r1(k1)=6;
    end
    
    if k1==3
        T(k1)=0;
    else
        T(k1)=T(k1-1)+1;
    end
    
   c(k1)=y1(k1-1);
   e(k1)=r1(k1)-c(k1);
  
       u1(k1)=(u1(k1-1))+(kp*(1+(ts/ti)+(td/ts))*e(k1))-((kp*(1+(2*td)/ts))*e(k1-1)+((kp*(td/ts))*e(k1-2)));
     
   %if 
  % del_u(k1)=32-u1(k1);
   y1(k1)=(0.1268*u1(k1))+(3.976e-16*y1(k1-1));
   %uusave(k1,:)=u1(k1);
   uusave(k1,:)=u1(k1)+12;
   yysave(k1,:)=y1(k1);
   es=((e(k1)).^2)%/(r1(k1)).^2);
   err=err+es;
   du(k1)=u1(k1)-u1(k1-1);
   duu=duu+((du(k1)).^2);
   rr1=[rr1,r1(k1)];
   
end

z=toc
for k1=1:101
if ((k1>=1)&&(k1<=40))
    sp=4;
else
    %sp=6;
    sp = 6;
end
spp=[spp;sp];
end

resp = stepinfo(yysave,T);

figure(1)
plot(T,spp,'r',T,yysave,'k','linewidth',2.25);
xlabel('time'), ylabel('Amplitude')
legend('setpoint','output response')
title('Comparison of setpoint and output response')
figure(2)
plot(T,uusave,'k','linewidth',2.25);
title('Control input')
xlabel('time'), ylabel('Amplitude')


    


