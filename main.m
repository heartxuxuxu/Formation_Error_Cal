close all;
clear all;
clc;
% now=datestr(now);
% str=[now '.txt'];
% fid=fopen(str,'wt+');
% filename=[num2str(datestr(now)),'error_data.txt'];
% fid=fopen(filename,'wt+');
fid = fopen('1_24_error_data.txt','at+');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%init%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
num=4;%参与编队的无人机初始数量
target_num=4;
%%%%%%%%%%%%%%%%%%%%%%%%quad_init_for_test%%%%%%%%%%%%%%%%%%%%%%%%%
% quad_init_x=[4.262191 10.562410 7.494362 7.488187];
% quad_init_y=[3.548897 0.896159 3.524472 2.816878];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
goal=[3 3;
      3 8;
      8 3;
      8 8];% 目标点位置 [x(m),y(m)]
temp_goal=goal;
goal_series=[1;2;3;4];
obstacleR=0.5;% 冲突判定用的障碍物半径
global dt; dt=0.1;% 时间[s]

% 机器人运动学模型
% 最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
% 速度分辨率[m/s],转速分辨率[rad/s]]
Kinematic=[1.0,toRadian(90.0),0.3,toRadian(50.0),0.01,toRadian(1)];
% 评价函数参数 [heading,dist,velocity,predictDT]
evalParam=[0.1,0.3,0.1,3.0 ];
area=[-1 12 -1 12];% 模拟区域范围 [xmin xmax ymin ymax]
mirror_dis=zeros(4,4);
% 模拟实验的结果
result.quad1=[];
result.quad2=[];
result.quad3=[];
result.quad4=[];
traj1=[];
traj2=[];
traj3=[];
traj4=[];
total_time=1;
error_time=0;
% Main loop
for test=1:total_time
    tic;
    %Target Allocation
    error_flag=0;
%     quad_init_x=12*rand(num,1);%初始位置生成
%     quad_init_y=12*rand(num,1);
    quad_init_x=[4.262191; 10.562410; 7.494362; 7.488187];
    quad_init_y=[3.548897; 0.896159; 3.524472; 2.816878];
    [goal]=Target_Allocation(goal,quad_init_x,quad_init_y,num,target_num,goal_series,temp_goal,mirror_dis);
    x=[quad_init_x(1,1) quad_init_y(1,1) atan2((goal(1,2)-quad_init_y(1,1)),(goal(1,1)-quad_init_x(1,1))) 0 0;
       quad_init_x(2,1) quad_init_y(2,1) atan2((goal(2,2)-quad_init_y(2,1)),(goal(2,1)-quad_init_x(2,1))) 0 0;
       quad_init_x(3,1) quad_init_y(3,1) atan2((goal(3,2)-quad_init_y(3,1)),(goal(3,1)-quad_init_x(3,1))) 0 0;
       quad_init_x(4,1) quad_init_y(4,1) atan2((goal(4,2)-quad_init_y(4,1)),(goal(4,1)-quad_init_x(4,1))) 0 0]';% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:5000
        % DWA参数输入
        obstacle1=[x(1,2) x(2,2);x(1,3) x(2,3);x(1,4) x(2,4)];
        obstacle2=[x(1,1) x(2,1);x(1,3) x(2,3);x(1,4) x(2,4)];
        obstacle3=[x(1,1) x(2,1);x(1,2) x(2,2);x(1,4) x(2,4)];
        obstacle4=[x(1,1) x(2,1);x(1,2) x(2,2);x(1,3) x(2,3)];

        if norm(x(1:2,1)-goal(1,:)')>0.1
            [x(:,1),traj1]=distributed_planning(x(:,1),Kinematic,goal(1,:),evalParam,obstacle1,obstacleR);
        end
        if norm(x(1:2,2)-goal(2,:)')>0.1
            [x(:,2),traj2]=distributed_planning(x(:,2),Kinematic,goal(2,:),evalParam,obstacle2,obstacleR);
        end
        if norm(x(1:2,3)-goal(3,:)')>0.1
            [x(:,3),traj3]=distributed_planning(x(:,3),Kinematic,goal(3,:),evalParam,obstacle3,obstacleR);
        end
        if norm(x(1:2,4)-goal(4,:)')>0.1
            [x(:,4),traj4]=distributed_planning(x(:,4),Kinematic,goal(4,:),evalParam,obstacle4,obstacleR);
        end

        % 模拟结果的保存
        result.quad1=[result.quad1; x(:,1)'];
        result.quad2=[result.quad2; x(:,2)'];
        result.quad3=[result.quad3; x(:,3)'];
        result.quad4=[result.quad4; x(:,4)'];
        % 是否到达目的地
        if norm(x(1:2,1)-goal(1,:)')<0.1 && norm(x(1:2,2)-goal(2,:)')<0.1 && norm(x(1:2,3)-goal(3,:)')<0.1 && norm(x(1:2,4)-goal(4,:)')<0.1
            str=[num2str(test) ' times Arrive Goal!!!'];
            disp(str);break;
        end
        if norm(x(1:2,1)-x(1:2,2))<obstacleR || norm(x(1:2,1)-x(1:2,3))<obstacleR || norm(x(1:2,1)-x(1:2,4))<obstacleR || norm(x(1:2,2)-x(1:2,3))<obstacleR || norm(x(1:2,2)-x(1:2,4))<obstacleR || norm(x(1:2,3)-x(1:2,4))<obstacleR
            error_flag=1;
            str=[num2str(test) ' times error!!!'];
            disp(str);break;
        end
        %====Animation====
        hold off;
        ArrowLength=0.5;%    
        plot(goal(1,1),goal(1,2),'*b');hold on;
        plot(goal(2,1),goal(2,2),'*m');hold on;
        plot(goal(3,1),goal(3,2),'*r');hold on;
        plot(goal(4,1),goal(4,2),'*c');hold on;
        quiver(x(1,1),x(2,1),ArrowLength*cos(x(3,1)),ArrowLength*sin(x(3,1)),'ok');hold on;
        quiver(x(1,2),x(2,2),ArrowLength*cos(x(3,2)),ArrowLength*sin(x(3,2)),'ok');hold on;
        quiver(x(1,3),x(2,3),ArrowLength*cos(x(3,3)),ArrowLength*sin(x(3,3)),'ok');hold on;
        quiver(x(1,4),x(2,4),ArrowLength*cos(x(3,4)),ArrowLength*sin(x(3,4)),'ok');hold on;    
        axis(area);
        grid on;
        drawnow;
        time=toc;
        if time>50
            error_flag=1;break;
        end
    end
    toc
    time=toc;
    close all;
    if time>30 || error_flag==1
        if time>1
            error_time=error_time+1;
        end
        fprintf(fid,'%f %f %f %f %f %f %f %f %f\n',time,quad_init_x(1,1),quad_init_x(2,1),quad_init_x(3,1),quad_init_x(4,1),quad_init_y(1,1),quad_init_y(2,1),quad_init_y(3,1),quad_init_y(4,1)); 
    end
end
accuracy_rate=1-error_time/total_time;
str=['accuracy_rate is ' num2str(accuracy_rate)];
disp(str);
% fclose(fid);