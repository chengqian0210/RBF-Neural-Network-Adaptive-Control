close all;

figure(1);
subplot(211);
plot(t,y(:,1),t,y(:,3),'linewidth',2);
xlabel('ʱ��');ylabel('λ��');
legend('Ŀ��λ��','ʵ��λ��');
subplot(212);
plot(t,y(:,2),t,y(:,4),'linewidth',2);
xlabel('ʱ��');ylabel('�ٶ�');
legend('Ŀ��ת��','ʵ��ת��');

figure(2);
plot(t,u(:,1),'linewidth',2);
xlabel('ʱ��');ylabel('��������');

figure(3);
subplot(211);
plot(t,p(:,1),t,p(:,4),'linewidth',2);
xlabel('ʱ��');ylabel('Fx');
legend('Fx��ʵֵ','Fx����ֵ');
subplot(212);
plot(t,p(:,2),t,p(:,5),'linewidth',2);
xlabel('ʱ��');ylabel('m');
legend('m��ʵֵ','m����ֵ');