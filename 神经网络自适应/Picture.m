close all;

figure(1);
subplot(211);
plot(t,y(:,1),t,y(:,3),'linewidth',2);
xlabel('时间');ylabel('位置');
legend('目标位置','实际位置');
subplot(212);
plot(t,y(:,2),t,y(:,4),'linewidth',2);
xlabel('时间');ylabel('速度');
legend('目标转速','实际转速');

figure(2);
plot(t,u(:,1),'linewidth',2);
xlabel('时间');ylabel('控制输入');

figure(3);
subplot(211);
plot(t,p(:,1),t,p(:,4),'linewidth',2);
xlabel('时间');ylabel('Fx');
legend('Fx真实值','Fx估计值');
subplot(212);
plot(t,p(:,2),t,p(:,5),'linewidth',2);
xlabel('时间');ylabel('m');
legend('m真实值','m估计值');