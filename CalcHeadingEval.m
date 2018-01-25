function heading=CalcHeadingEval(x,goal)
% heading�����ۺ�������
theta=toDegree(x(3));% �����˳���

goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% Ŀ���ķ�λ

        
if goalTheta>theta
    targetTheta=goalTheta-theta;% [deg]
else
    targetTheta=theta-goalTheta;% [deg]
end

heading=180-targetTheta;