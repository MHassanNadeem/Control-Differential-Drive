close all;
clear all;
clc;

noise = 0;
closeLoop = 1;

%% Path Generation
% elipse
start = 0;
stop = 5*pi;

y = sin(start:0.01:stop);
y2 = -y;

x = start:0.01:stop;

yr = [y]*10;
xr = [x];

xr=xr;
yr=yr;

%% Initializing Robot at the start with correct orientation
robot.len = 0.5;
robot.x = xr(1)-0.5;
robot.y = yr(1);
robot.th = atan2((yr(2)-yr(1)),(xr(2)-xr(1)))+0.8;
robot.path.x = robot.x;
robot.path.y = robot.y;
robot.path.th = robot.th;
robot.x_d = 0;
robot.y_d = 0;
robot.th_d = 0;
robot.v = 0;
robot.w = 0;

%% Ploting Refernce Path and Robot Orientation
figure; hold on; grid on; axis([-1 16 -12 12]);
plot(xr,yr,'r:','LineWidth',3.5);
robo = plot(0,0,'k','LineWidth',7);
roboPath = plot(0,0,'b','LineWidth',2);
set(robo,'xdata',[robot.x robot.x+cos(robot.th)*robot.len],'ydata',[robot.y robot.y+sin(robot.th)*robot.len]);
pause(1);

errorRef.x = 0;
errorRef.y = 0;
errorRef.th= 0;
error.x_old = 0;
error.y_old = 0;
error.th_old= 0;
error.x = 0;
error.y = 0;
error.th= 0;
error.x_d=0;
error.y_d=0;
error.v = 0;
error.w= 0;

k.x = 2*.6114*3.7620;
k.y = 2;
k.th = 2*.6114*3.7620;

% k.x = 0;
% k.y = 0;
% k.th = 0;


%% Trajectory Tracking
xr_d = 0;
yr_d = 0;
xr_d_Old = 0;
yr_d_Old = 0;
dt = 0.01;
wff =0;
vff =0;
for i=1:(length(xr)-2)
    thR = atan((yr(i+1)-yr(i)) / (xr(i+1)-xr(i)));
%     thR = atan2((yr(i+1)-yr(i)),(xr(i+1)-xr(i)));
    if(xr(i+1)<xr(i))
        thR = -pi+abs(thR);
        if(yr(i+1)>yr(i))
            thR = -(pi+abs(atan((yr(i+1)-yr(i)) / (xr(i+1)-xr(i)))));
        end
    end
%     if(thR == 0)
%         thR = -pi/2;
%     end

    
    errorRef.x = xr(i) - robot.x;
    errorRef.y = yr(i) - robot.y;
    errorRef.th= thR - robot.th;
%     display(robot.th);
%     if(robot.th>pi)
%         robot.th = 
%     display(thR);
%  display([num2str(thR) '          ' num2str(robot.th) '       ' num2str(error.th)]);
%  if(error.th>1)
%      break;
%  end
    
    error.x_old = error.x;
    error.y_old = error.y;
    error.th_old = error.th;
    
    error.x = cos(robot.th)*errorRef.x + sin(robot.th)*errorRef.y;
    error.y = - sin(robot.th)*errorRef.x + cos(robot.th)*errorRef.y;
    error.th = errorRef.th;
    
%     display(error.th);
    
    error.x_d_old = error.x_d;
    error.y_d_old = error.y_d;
    
    error.x_d = error.x - error.x_old;
    error.y_d = error.y - error.y_old;
    
    error.x_d_d = error.x_d - error.x_d_old;
    error.y_d_d = error.y_d - error.y_d_old;
    
    error.v = sqrt(error.x^2 + error.y^2);
    error.w = (error.x_d*error.y_d_d - error.y_d*error.x_d_d)/(error.x_d^2 + error.y_d^2);
    
    if(isnan(error.w))
           display('BAD 1 ____________________');
          error.w = 0;
    end
%      display(error.w);
%      display(wff);
    
    xr_d_Old = xr_d;
    yr_d_Old = yr_d;
    xr_d = (xr(i+1)-xr(i))/dt;
    yr_d = (yr(i+1)-yr(i))/dt;
    
%     if(i>1)
    xr_d_d = (xr_d - xr_d_Old)/dt;
    yr_d_d = (yr_d - yr_d_Old)/dt;
%     end
%     xr_d_d =0 ;
%     yr_d_d = 0;

    vff = sqrt(xr_d^2 + yr_d^2);
    wff = (xr_d*yr_d_d - yr_d*xr_d_d)/(xr_d^2 + yr_d^2);
    
    robot.v = vff + k.x*error.x;
    robot.w = wff + k.y*vff*(sign(vff))*error.y + k.th*error.th;
%     display(error.th);
% %     display(error.th);
% error.th;
% display([num2str(error.x) '          ' num2str(error.y) '           ' num2str(error.th)]);

    if(closeLoop)
        robot.x_d = cos(robot.th)*(robot.v);
        robot.y_d = sin(robot.th)*(robot.v);
        robot.th_d = robot.w;
    else
        robot.x_d = cos(robot.th)*(vff);
        robot.y_d = sin(robot.th)*(vff);
        robot.th_d = wff;
    end


    if(isnan(robot.th_d))
        display('____________________________________________');
        robot.th_d = 0;
%         break;
%         i=i+5;
    end

%     Calcualte and Plot Robot
    robot.x = robot.x + robot.x_d*dt;
    robot.y = robot.y + robot.y_d*dt;
    robot.th = robot.th + robot.th_d*dt;
    
%     Adding Noise
    if(noise)
        if(rand<0.3)
            robot.x = robot.x + ((rand-0.5)/5);
            robot.x = robot.x + ((rand-0.5)/5);
        end
    end
    
    

    
    if(isnan(thR))
        thR = -pi/2;
    end

    set(robo,'xdata',[robot.x robot.x+cos(robot.th)*robot.len],'ydata',[robot.y robot.y+sin(robot.th)*robot.len]);
    
%     Save and Plot Robot Path
    robot.path.x = [robot.path.x robot.x];
    robot.path.y = [robot.path.y robot.y];
    robot.path.th = [robot.path.th robot.th];
    set(roboPath,'xdata',robot.path.x,'ydata',robot.path.y);
    
    k.y = ((3.7620^2-wff^2)/abs(vff));
     pause(dt);
end

set(robo,'xdata',[robot.x robot.x+cos(robot.th)*robot.len],'ydata',[robot.y robot.y+sin(robot.th)*robot.len]);
set(roboPath,'xdata',robot.path.x,'ydata',robot.path.y);
