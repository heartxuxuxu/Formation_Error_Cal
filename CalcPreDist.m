function Predist=CalcPreDist(x,ob,R)
%CALCPREDIST Summary of this function goes here
%   Detailed explanation goes here
global dt;
Predist=100;
pre_ob(:,:)=ob(:,:);
F_ob = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

for io=1:length(ob(:,1))
    B_ob = [dt*cos(pre_ob(io,3))*pre_ob(io,4);
            dt*sin(pre_ob(io,3))*pre_ob(io,4);
            0;
            0];
    for time=0:dt:3.0
        pre_ob(io,:)=(F_ob*pre_ob(io,:)'+B_ob)';%cos(pre_ob(3))
    end
    Predisttmp=norm(pre_ob(io,1:2)-x(1:2)')-R;%�p�X�̈ʒu��?�Q���Ƃ̃m������?����v�Z
    if Predist>Predisttmp% ���ϰ�����С�ľ���
        Predist=Predisttmp;
    end
    if pre_ob(io,4)<0.2
        Predist=3*R;
    end
end

% �ϰ�����������޶�һ�����ֵ��������趨��һ��һ���켣û���ϰ����̫ռ����
if Predist>=3*R
    Predist=3*R;
end

end

