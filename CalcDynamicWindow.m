function Vr=CalcDynamicWindow(x,model)
%
global dt;
% �����ٶȵ������С��Χ
Vs=[0 model(1) -model(2) model(2)];

% ���ݵ�ǰ�ٶ��Լ����ٶ����Ƽ���Ķ�̬����
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

% ���յ�Dynamic Window
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];