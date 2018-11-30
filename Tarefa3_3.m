%modelagem do robo móvel

clc; clear; close all; warning off;

%Tempo de simulação
tfin=20;
%Amostragem
To=0.1;
%Tempo
t= [0:To:tfin];

% Matrizes constantes positivas definidas (mudar se necessário)
l = 0.8;
kp = 0.5;
L = diag([l,l,l,l]);
Kp = diag([kp,kp,kp,kp]);
a=[0.2;0.2];

%Posições iniciais
x1 = -3;
y1 = 4;
x2 = -2;
y2 = 6;

X = [x1 y1 x2 y2];

%Condições iniciais robô 1
x(1) = X(1); %inicial em x [m]
y(1) = X(2); %inicial em y [m]
psi(1) = 0; %Angulo inicial [rad]

%Condições iniciais robô 2
x2(1) = X(3); %inicial em x [m]
y2(1) = X(4); %inicial em y [m]
psi2(1) = 0; %Angulo inicial [rad]

%Posições finais
xf = 0;
yf = 0;
rof = 1;
alfaf = pi/2;
qd = [xf yf rof (pi/2 - alfaf)];

%Referência desejada, apenas para plotar
Pxd=qd(1);   % Desejada em X
Pyd=qd(2);   % Desejada em Y

%%
%CONTROLADOR
for k=1: length(t)
      
      % Controlador cinemático (como no slide 6 do conjunto 10).
      invK = [cos(psi(k)), sin(psi(k)),0,0;
          -(1/a(1))*sin(psi(k)),(1/a(1))*cos(psi(k)),0,0;
          0,0,cos(psi2(k)),sin(psi2(k));
          0,0,-(1/a(2))*sin(psi2(k)),(1/a(2))*cos(psi2(k))];
      
      % q = xf, yf, pf, alfaf (como especificado pelo trabalho). Cálculos
      % como no slide 4 do conjunto 10, com exceção da posição, que
      % coincidem com a do robô 1 no trabalho.
      q(:,k) = [x(k), y(k), sqrt(abs(x(k)-x2(k))^2 + abs(y(k)-y2(k))^2), atan((y(k) - y2(k))/(x(k)- x2(k) +eps))];
      
      % Jacobiana, ver se da certo usar assim, se não der, existe um exemplo
      % da matriz no slide 5 do conjunto 10.
      invJ = [1,0, 0, 0;
          0,1, 0, 0;
          1,0, cos(q(4,k)), -q(3)*sin(q(4,k));
          0,1, sin(q(4,k)), q(3)*cos(q(4,k))];
      
      % Cálculo da velocidade da formação. (como no slide 6 do conjunto 10)
      qpontoref(:,k) = L*tanh(inv(L)*Kp*(qd-q(:,k)')');
      
      % Cálculo de vref (slide 3 do conjunto 10)
      vref(:,k) = invK*invJ*qpontoref(:,k);

      u_ref(k) = vref(1,k);   %velocidade linear de control[m/s] 1
      w_ref(k) = vref(2,k);   %velocidade angular[rad/seg] 1
    
      u_ref2(k) = vref(3,k);   %velocidade linear de control[m/s] 2
      w_ref2(k) = vref(4,k);   %velocidade angular[rad/seg] 2
      
      %Modelo cinemático 1
      xp(k) =u_ref(k)*cos(psi(k))-a(1)*w_ref(k)*sin(psi(k));
      yp(k) =u_ref(k)*sin(psi(k))+a(1)*w_ref(k)*cos(psi(k));
      
      %Modelo cinemático 2
      xp2(k) =u_ref2(k)*cos(psi2(k))-a(2)*w_ref2(k)*sin(psi2(k));
      yp2(k) =u_ref2(k)*sin(psi2(k))+a(2)*w_ref2(k)*cos(psi2(k));
   
      psi(k+1) = To*w_ref(k)+psi(k);
      psi2(k+1) = To*w_ref2(k)+psi2(k);
        
      x(k+1) =To*xp(k)+x(k);
      y(k+1) =To*yp(k)+y(k);       
      x2(k+1) =To*xp2(k)+x2(k);
      y2(k+1) =To*yp2(k)+y2(k);       
end

%Animação
scrsz=get(0,'ScreenSize');
off1=0;
off2=10;
figpos=[off1 off2 scrsz(3)-off1 scrsz(4)-off2];

f1=figure(1);
set(f1,'Color','w','Position',figpos);

paso=2; axis 'equal'

Robot_Dimension(2);
Ho=Robot_Plot_3D(x(1),y(1),psi(1),'g'); hold on
Ho2=Robot_Plot_3D(x2(1),y2(1),psi2(1),'g'); hold on

H1 = plot(x(1),y(1),'*m'); hold on;
H2 = plot(x(1),y(1),'*m'); hold on;
H4 = plot(Pxd(1),Pyd(1),'b');

H12 = plot(x2(1),y2(1),'*m'); hold on;
H22 = plot(x2(1),y2(1),'*m'); hold on;
H42 = plot(Pxd(1),Pyd(1),'b');

for i=1:paso:length(t)
    
    delete(Ho)
    delete(H1)
    delete(H2)
    delete(H4)
    delete(Ho2)
    delete(H12)
    delete(H22)
    delete(H42)
    axis([-8 8 -8 8 0 4]);  
    view([-30.0,30.0]);
    Ho=Robot_Plot_3D(x(i),y(i),psi(i),'g');hold on
    H1 = plot(x(i),y(i),'*m');hold on
    H2 = plot(x(1:i),y(1:i),'g');hold on
    H3 = plot(Pxd,Pyd,'m');hold on
    H4 = plot(Pxd,Pyd,'*b');
    Ho2=Robot_Plot_3D(x2(i),y2(i),psi2(i),'b');hold on
    H12 = plot(x2(i),y2(i),'*m');hold on
    H22 = plot(x2(1:i),y2(1:i),'g');hold on
    H32 = plot(Pxd,Pyd,'m');hold on
    H42 = plot(Pxd,Pyd,'*b');
    grid on;
    pause(To)
end

%Gráficos Plotados apenas para Robô 1
%
figure
subplot(3,1,1)
plot(t,x(1:length(t))); grid;
title('Deslocamento em X');
xlabel('Tempo[s]'); ylabel('X[m]');

subplot(3,1,2)
plot(t,y(1:length(t)),'g'); grid;
title('Deslocamento em Y');
xlabel('Tempo[s]'); ylabel('Y[m]');

subplot(3,1,3)
plot(t,psi(1:length(t)),'m'); grid;
title('Rotação');
xlabel('Tempo[s]'); ylabel('\psi')

figure
plot(x,y); grid;
title('Deslocamento em XY');
xlabel('X[m]'); ylabel('Y[m]');

%Gráficos para verificação dos resultados

figure
subplot(4,1,1)
plot(t,q(1,1:length(t)),'linewidth',2); grid;
title('Erro de X');
xlabel('Tempo[s]'); ylabel('X[m]');

subplot(4,1,2)
plot(t,q(2,1:length(t)),'g','linewidth',2); grid;
title('Erro de Y');
xlabel('Tempo[s]'); ylabel('Y[m]');

subplot(4,1,3)
plot(t,q(3,1:length(t)),'m','linewidth',2); grid;
title('Valor de \rho');
xlabel('Tempo[s]'); ylabel('\rho [m]')

subplot(4,1,4)
plot(t,(q(4,1:length(t))+pi/2),'c','linewidth',2); grid;
title('Valor de \alpha');
xlabel('Tempo[s]'); ylabel('\pi rad')


