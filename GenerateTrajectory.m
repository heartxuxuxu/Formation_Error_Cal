function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
% �켣���ɺ���
% evaldt��ǰ��ģ��ʱ��; vt��ot��ǰ�ٶȺͽ��ٶ�; 
global dt;
time=0;
u=[vt;ot];% ����ֵ
traj=x;% �����˹켣
while time<=evaldt
    time=time+dt;% ʱ�����
    x=f(x,u);% �˶�����
    traj=[traj x];
end