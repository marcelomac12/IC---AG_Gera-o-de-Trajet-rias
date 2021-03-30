%%ManipuladorTeste2


clear all;
close all;
clc

L=[10,10,10];
xf = [20; 20];
q = [0,0307; 1,8756; 1,5691];
%hold on
plot(xf(1),xf(2),'ob');
%text(xf(1)+3,xf(2),'Target');
plot(q,L)
pause(2)

%%%%% Parametros do AG
nind = 80;
vars = 3;
range = [-pi -pi -pi; pi pi pi];
gap = 0.98;
gen = 0;
erro = 10;

%%%%% Populaçao inicial e Avaliaçao
pop = crtrp(nind, range);% a função crtrp cria uma população inicial aleatória  
f = fit3(pop,xf);
    %fitcubic
tic
tmp=cputime;

%%ManipuladorTeste1 - Define o robo
d1 = 0;
d2 = 0;
d3 = 0;
theta1 = 0;
theta2 = 0;
theta3 = 0;
a1 = 50;
a2 = 40;
a3 = 30;
alpha1 = 0;
alpha2 = 0;
alpha3 = 0;

L1(1) = Link([0 d1 a1 alpha1]);
L1(2) = Link([0 d2 a2 alpha2]);
L1(3) = Link([0 d3 a3 alpha3]);

Robo = SerialLink(L1, 'name', 'teste');

%%%%% Geraçoes
while gen < 250,%% Evolução
F = scaling(f);
s = select('sus', pop, F, gap);
s = recombin('xovsp',s,0.8);
s = mutbga(s,range,0.03);
fs = fit3(s,xf);
    %fitcubic
[pop f]=reins(pop,s,1,1,f,fs);
gen = gen+1;

%%%%% Escolha do melhor elemento
[m,indm] = max(f);
[p,indp] = min(f);
t1 = pop(indm,1);
t2 = pop(indm,2);
t3 = pop(indm,3);
the = [t1;t2;t3];
qf = the;

%%%%% Cinemática Direta
%cL=L.*cos(qf);
%sL=L.*sin(qf);
%%%%% posição do end-effector
%x=[sum(cL);sum(sL)];
%[x]=kinmodel(qf,L);


T = Robo.fkine(qf);
x = T.transl;

%%%%% Calculo do erro
erro = sqrt((xf(1) - x(1))^2 + (xf(2) - x(2))^2);
et = sqrt((t1 - q(1))^2 + (t2 - q(2))^2 + (t3 - q(3))^2);

%%%%% Desenha a posiçao atual do robo
title(['Geraçao: ',num2str(gen), ' Erro: ',num2str(erro)]);
%plotrobot(the,L1)

%%%%% Estatisticas
de(gen,1) = erro; %erro de posiçao
dt(gen,1) = et; %menor deslocamento de juntas
mf(gen,1) = m; %melhor fitness
pf(gen,1) = p; %pior fitness
fm(gen,1) = sum(f)/nind; %fitness medio da geraçao
end

%%%%% Trajetória Cúbica
tf = 5;
pts = 9;
t = linspace(0,5,pts);

%tic
%tmp=cputime;
for i = 1:3
for k = 1:pts;
qtrj(i,k) = q(i) + (3/(tf^2))*(qf(i) - q(i))*t(k)^2 - (2/(tf^3))*(qf(i) - q(i))*t(k)^3;
end
end
for i = 1:pts
%pos(:,i) = kinmodel(qtrj(:,i),L);
pos(:,i) = Robo.fkine(qtrj(:,i),L);
end

qtrj = qtrj';
elos=[10 10 10];
th = [qtrj(:,1) qtrj(:,2) qtrj(:,3)];
%pos = [x y];
toc
cputime-tmp
qf
grafico(pos',th,elos,xf)