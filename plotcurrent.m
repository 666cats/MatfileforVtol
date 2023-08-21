clc
figure(1);
plot(0.5+10:0.5:59*0.5+10,current.current_a0,'-b',LineWidth=1)
hold on
plot(0.5+10:0.5:59*0.5+10,current.current_a1,'-r',LineWidth=1)
xlabel('时间/s')
ylabel('电流/A')
title('悬停和巡航飞行电流')
legend('巡航飞行','悬停飞行')