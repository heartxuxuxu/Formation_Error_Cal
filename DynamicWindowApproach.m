function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)% DWA��������

% Dynamic Window [vmin,vmax,wmin,wmax]
Vr=CalcDynamicWindow(x,model);

% ���ۺ����ļ���
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    disp('Initial Error!!');
    u=[0;0];return;
end

% �����ۺ�������
evalDB=NormalizeEval(evalDB);

% �������ۺ����ļ���
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[maxv,ind]=max(feval);% �������ۺ���
u=evalDB(ind,1:2)';% 
