function stopDist=CalcBreakingDist(vel,model)
% �����˶�ѧģ�ͼ����ƶ�����,����ƶ����벢û�п�����ת�ٶȣ�����ȷ�ɣ�����
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;% �ƶ�����ļ���
    vel=vel-model(3)*dt;% 
end