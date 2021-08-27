%% This matlab script shows some variables of a txt file.
% @author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado" (YouTube channel)

clc, close, clear

%Load the txt file
D = load('/home/mau/data_py.txt'); %Default path of the data file

%Extract every signal from file
t = D(:,1);
p0 = 2; pf = length(t); t=t(p0:pf); %This is to omit initial values

Xd = D((p0:pf),2); Yd = D((p0:pf),3);
x = D((p0:pf),4);  y = D((p0:pf),5);
V = D((p0:pf),6);  W = D((p0:pf),7);

figure(1)   %Trajectories on the horizontal plane and control signals ---------------------
%subplot(2,1,1) %rows = 2, cols = 1, position = 1
plot(Xd(:),Yd(:),'--k','LineWidth',1.5);
hold on, grid on, axis equal, axis([-1.2 1.2 -1.2 1.2])
title('Trajectories on the horizontal plane')
xlabel('X_G [m]'), ylabel('Y_G [m]')
plot(x(:),y(:),'--b','LineWidth',1.5);
legend('R_d','R') %Latex notation can be used

figure(2)
%subplot(2,1,2)
plot(t,V(:),'k','LineWidth',1.2)
grid on, hold on
plot(t,W(:),'r','LineWidth',1.2)
legend('V','\omega')
title('Velocities of the mobile robot')
xlabel('Time [s]'), ylabel('V, W [m/s, rad/s]')

