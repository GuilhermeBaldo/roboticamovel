%% Tarefa 1
clear
clc
close all

%% Define o tempo de amostragem e o tempo de execu��o da tarefa
ts = 0.1; % amostragem de 100ms
t = 0:ts:40; % tempo de tarefa de 40s

%% Define as vari�veis do modelo cinem�tico
a = .2; % ponto de controle deslocado de 20cm
kpx = 0.2; % constante kpx do controlador 
kpy = 0.2; % constante kpy do controlador

%% Define as vari�veis de pose, velocidade desejada, velocidade de refer�ncia, velocidade do rob� e erro de velocidade
pose = zeros(3,length(t)); % pose (x, y, psi)
vD = zeros(2,length(t)); % velocidade u, w
vRobot = zeros(2,length(t)); % velocidade u, w

%% Define vari�veis auxiliares para o plot do rob� 3D
theta=[0.2604 0.2509 -0.000499 0.9965 0.00263 1.07680];
H=[theta(1) 0;0 theta(2)];

%% Define a trajet�ria no tempo

xd = 2*ones(1,length(t)); % x desejado da trajet�ria em fun��o do tempo
yd = 1*ones(1,length(t)); % y desejado da trajet�ria em fun��o do tempo

xD = [xd;yd];   % vari�vel com todo os trajeto

xd_ponto=zeros(1,length(t)); % derivada temporal de x da trajet�ria
yd_ponto=zeros(1,length(t)); % derivada temporal de y da trajet�ria

%% inicio impress�o
[x,y,psi,~,~,~,~] = leerRobot();
scrsz=get(0,'ScreenSize');
off1=0;
off2=10;
figpose=[off1 off2 scrsz(3)-off1 scrsz(4)-off2];

f1=figure(1);
set(f1,'Color','w','position',figpose);

paso=2; axis 'equal'
Robot_Dimension(2);
Ho=Robot_Plot_3D(x,y,psi,'g'); hold on

H1 = plot(x,y,'*m'); hold on;
H2 = plot(x,y,'*m'); hold on;

%% executa o loop de controle pelo tempo definido
for i = 2:length(t)
    
    %% Leitura dos dados do rob�
    [pose(1,i),pose(2,i),pose(3,i),vRobot(1,i),vRobot(2,i),~,~] = leerRobot();
    
    %% impress�o atualizada
    delete(Ho)
    delete(H1)
    delete(H2)
    axis([-4 4 -4 4 0 3]);
    view([-30.0,30.0]);
    Ho=Robot_Plot_3D(pose(1,i),pose(2,i),pose(3,i),'g');hold on
    H1 = plot(pose(1,i),pose(2,i),'*m');hold on
    H2 = plot(pose(1,1:i),pose(2,1:i),'g');hold on
    H3 = plot(xd,yd,'m');hold on
    grid on;
     
    %% Controlador cinem�tico
    invA = [cos(pose(3,i)) sin(pose(3,i)); ...
        -(1/a)*sin(pose(3,i)) (1/a)*cos(pose(3,i))];
    
    vD(:,i) = invA*[xd_ponto(i) + tanh(kpx*(xd(i)-pose(1,i)));...
        yd_ponto(i)+ tanh(kpy*(yd(i)-pose(2,i)))];
        
    %% Envia a velocidade para o rob�
    escribirRobot(vD(1,i),vD(2,i));
    
    %% Pausa 100ms (tempo de amostragem)
    pause(ts);
end

%% Ao final, para o rob�
escribirRobot(0,0);

%% Erros
pErro = [pose(1,:) - xD(1,:);pose(2,:) - xD(2,:)];
vErro = [vRobot(1,:) - vD(1,:);vRobot(2,:) - vD(2,:)];

figure
ax1 = subplot(2,1,1);
plot(t,pErro(1,:),'linewidth',2);hold on;
plot(t,pErro(2,:),'linewidth',2);
ylabel('[m]');
xlabel('tempo [s]');
legend('Erro em X','Erro em Y','location','best');
legend('boxoff');
grid minor;
title('Erro de posi��o');

ax2 = subplot(2,1,2);
plot(t,vErro(1,:),'linewidth',2);hold on;
ylabel('[m/s]');
yyaxis right
yticks([-1.5*pi -1*pi -0.5*pi 0 0.5*pi 1*pi 1.5*pi]);
yticklabels({'-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi'})
ylabel('[Rad/s]');
xlabel('tempo [s]');
plot(t,vErro(2,:),'linewidth',2);
legend('Erro de Velocidade Linear','Erro de Velocidade Angular','location','best');
legend('boxoff');
grid minor;
title('Erro de velocidade');
linkaxes([ax1 ax2],'x');

IAEp = zeros(length(t),2);
IAEv = zeros(length(t),2);

for i=1:length(t)
    IAEp(i,1) = sum(abs(pErro(1,1:i)));
    IAEp(i,2) = sum(abs(pErro(2,1:i)));
    IAEv(i,1) = sum(abs(vErro(1,1:i)));
    IAEv(i,2) = sum(abs(vErro(2,1:i)));
end

figure
ax1 = subplot(2,1,1);
plot(t,IAEp(:,1),'linewidth',2);hold on;
plot(t,IAEp(:,2),'linewidth',2);
ylabel('[m]');
xlabel('tempo [s]');
legend('IAE em X','IAE em Y','location','best');
legend('boxoff');
grid minor;
title('IAE de posi��o');

ax2 = subplot(2,1,2);
plot(t,IAEv(:,1),'linewidth',2);hold on;
ylabel('[m/s]');
ylim([0 15]);
yyaxis right
yticks([0 10*pi 20*pi 30*pi]);
yticklabels({'0','10\pi','20\pi','30\pi'})
ylabel('[Rad/s]');
xlabel('tempo [s]');
plot(t,IAEv(:,2),'linewidth',2);
legend('IAE de Velocidade Linear','IAE de Velocidade Angular','location','south');
legend('boxoff');
grid minor;
title('IAE de velocidade');
linkaxes([ax1 ax2],'x');

%% Salva as vari�veis do rob� durante o experimento e plota
save('Tarefa2_Pioneer','pose','xD','vRobot','vD','pErro','IAEp','vErro','IAEv')
