function dist=CalcDistEval(x,ob,R)
% �ϰ���������ۺ���

dist=100;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;%�p�X�̈ʒu�Ə�Q���Ƃ̃m�����덷���v�Z
    if dist>disttmp% ���ϰ�����С�ľ���
        dist=disttmp;
    end
end

% �ϰ�����������޶�һ�����ֵ��������趨��һ��һ���켣û���ϰ����̫ռ����
if dist>=2*R
    dist=2*R;
end