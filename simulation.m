%% Simulation
% This script simulates random ball throw trajectories, adds random Gaussian noise to the trajectories 
% and preforms Kalman Filter/Extended Kalman Filter on those synthetic noisy measurements.  
 clear all; close all; clc;
%% Initilization

sParams.sampPrecet = 0.6;
sParams.R = 0.1205;
sParams.weight = 0.6;
BallInitial_X = 0 + 0.1; %7.24 (4.57) % [m], distance from the basket to the 3-point (free-throw) line
BallInitial_Y = 1.91 + 0.5;  % [m]
sParams.BallInitial_Y = 1.8 + 0.4;
sParams.BallInitial_X = 0.1;

[V0,Angle,R,Weight] =  ParametersInitialization('free_throw'); % 3Pt,free_throw

g = -9.813; % [m/sec^2], gravity constant
sParams.g = g;
t = (0:1e-3:3); % [sec], time vector, process "frequency" is 1kHz
dt_TV = 1e-3;
dt_C = 1/50;
sParams.dt_C = dt_C;
indicesT = (1:dt_C/1e-3:3001); % [indices], samples time vector, sampeling period of 0.05[sec]
mu = 0; % [m], mean of daviation noise
sigma_x = 0.05; % [m], mean of daviation noise in the x axis
sParams.sigma_x = sigma_x;
sigma_y = 0.05; % [m], mean of daviation noise in the y axis
sParams.sigma_y = sigma_y;
sParams.sigma_w_KF = 7; % process noise std
sParams.sigma_w_EKF = 0.5; % process noise std
sParams.Pcov = 500; % initial state noise variance
dragCoeff = 0.47; % for a sphere
sParams.dragCoeff = dragCoeff;
crossSection = pi*R^2; % for a sphere
sParams.crossSection = crossSection;
rho = 1.225; % [kg/m^3] mass density of air
sParams.rho = rho;


dist = 0; %7.24 (4.57) % [m], distance from the basket to the 3-point (free-throw) line 
BasketHight = 3.05; % [m]
ThrowerHight = 1.91; % [m]

%% Projectiles
% projectile movment equetions 
X0 = -dist; % basket location is (0,BasketHight)
Y0 = ThrowerHight;
Vx0 = V0 * cosd(Angle);
Vy0 = V0 * sind(Angle);

% without drag 
pos_wo_now = [X0;Y0];
vel_wo_now = [Vx0;Vy0];

pos_wo_all = [pos_wo_now];
vel_wo_all = [vel_wo_now];

for ii=1:numel(t)
   v = norm(vel_wo_now);
   acc_temp= [0;g];
   vel_wo_new = vel_wo_now+acc_temp*dt_TV;
   pos_wo_new = pos_wo_now+(vel_wo_new+vel_wo_now)/2*dt_TV;
   pos_wo_now = pos_wo_new;
   vel_wo_now = vel_wo_new;
   pos_wo_all = [pos_wo_all,pos_wo_now];
   vel_wo_all = [vel_wo_all,vel_wo_now];
end
X_wo = pos_wo_all(1,:);
Y_wo = pos_wo_all(2,:);


 
 % with drag 
pos_now = [X0;Y0];
vel_now = [Vx0;Vy0];

pos_all = [pos_now];
vel_all = [vel_now];

for ii=1:numel(t)
   v = norm(vel_now);
   acc_temp= -0.5*dragCoeff*crossSection*rho*v*vel_now/Weight+[0;g];
   vel_new = vel_now+acc_temp*dt_TV;
   pos_new = pos_now+(vel_new+vel_now)/2*dt_TV;
   pos_now = pos_new;
   vel_now = vel_new;
   pos_all = [pos_all,pos_now];
   vel_all = [vel_all,vel_now];
end
X = pos_all(1,:);
Y = pos_all(2,:);

[~,i1] = max(Y);
[~,i2] = max(Y_wo);


%linear and non-linear models comparison

% figure();
% hold on;
% plot(X,Y,'r','LineWidth',4);
% scatter(X(i1),Y(i1),'r','LineWidth',4);
% plot(X_wo,Y_wo,'g','LineWidth',4);
% scatter(X_wo(i2),Y_wo(i2),'g','LineWidth',4);
% plot(4.57-(0:0.46:0.46),[3.05,3.05],'b','LineWidth',5);
% legend({'non-linear model(with drag)','max height of non-linear model','linear model(without drag)','max height of linear model','basket'},'FontSize',20);
% title('linear and non-linear models comparison',FontSize=18)
% xlabel('horizontal position [meter]',FontSize=14);
% ylabel('vertical position [meter]',FontSize=18);
% axis([0 6 0 5]);
% grid;
% grid minor;
% 
% hold off;

%% Projectile estimation with drag
Y_sampled = Y(indicesT); % projectile sampling
Y_sampled_noised = Y_sampled + normrnd(mu, sigma_y,size(indicesT));
X_sampled = X(indicesT);
X_sampled_noised = X_sampled + normrnd(mu, sigma_x,size(indicesT));
mask = (Y_sampled >= 0);
Y_sampled_noised = Y_sampled_noised(mask);
X_sampled_noised = X_sampled_noised(mask);


% EKF with drag
 % Graphs
figure()
hold on;
plot(X_sampled(mask),Y_sampled(mask),'LineWidth',4);
scatter(X_sampled_noised,Y_sampled_noised,40,'filled');
title('Projectile along the XY plane (with drag)- EKF - where the basket is located at (4.57,3.05)','FontSize',20);

predictions_EKF = EKF_Auxilary(Y_sampled_noised,X_sampled_noised,sParams);
EKF_vLocation_x = [];
EKF_vLocation_y = [];
EKF_vVelocity_x = [];
EKF_vVelocity_y = [];
for ii = 1 : numel(predictions_EKF)
    vTmp = predictions_EKF(ii).x;
    EKF_vLocation_x = [EKF_vLocation_x; vTmp(1)];
    EKF_vLocation_y = [EKF_vLocation_y; vTmp(2)];
    EKF_vVelocity_x = [EKF_vVelocity_x; vTmp(3)];
    EKF_vVelocity_y = [EKF_vVelocity_y; vTmp(4)];
end 
plot(EKF_vLocation_x,EKF_vLocation_y,'g','LineWidth',4);
plot(4.57-(0:0.46:0.46),[3.05,3.05],'b','LineWidth',5);
xlabel('horizontal position [meter]',FontSize=18);
ylabel('vertical position [meter]',FontSize=18);
flagPt = max(find([predictions_EKF.Flag]))+1;
scatter(EKF_vLocation_x(flagPt),EKF_vLocation_y(flagPt),50,'+','black',LineWidth=20);
legend({'real Projectile with drag','sensor Projectile measurements','EKF estimation','basket','last measurement for prediction'},'FontSize',20);
axis([0 6 0 4.5]);
grid;
grid minor;
hold off;



% EKF without drag
% figure()
% hold on;
% plot(X_sampled(mask),Y_sampled(mask),'LineWidth',3);
% scatter(X_sampled_noised,Y_sampled_noised,40,'filled');
% title('Projectile along the XY plane (without drag)- where the basket is located at (4.57,3.05)','FontSize',20);
% plot(4.57-(0:0.46:0.46),[3.05,3.05],'b','LineWidth',5);
% xlabel('horizontal position [meter]',FontSize=25);
% ylabel('vertical position [meter]',FontSize=25);
% legend({'real Projectile without drag','sensor Projectile measurements','basket'},'FontSize',15);
% axis([0 6 0 4.5]);
% grid;
% grid minor;
% hold off;


% % ordinary KF
% with drag
figure()
hold on;    
plot(X_sampled(mask),Y_sampled(mask),'LineWidth',4);
scatter(X_sampled_noised,Y_sampled_noised,40,'filled');
title('Projectile along the XY plane (with drag) - standard KF - where the basket is located at (4.57,3.05)','FontSize',20);

predictions_KF = KalmanfilterAuxilary(Y_sampled_noised,X_sampled_noised,sParams);
vLocation_x = [];
vLocation_y = [];
vVelocity_x = [];
vVelocity_y = [];
for ii = 1 : numel(predictions_KF)
    vTmp = predictions_KF(ii).x;
    vLocation_x = [vLocation_x; vTmp(1)];
    vLocation_y = [vLocation_y; vTmp(2)];
    vVelocity_x = [vVelocity_x; vTmp(3)];
    vVelocity_y = [vVelocity_y; vTmp(4)];
end 
plot(vLocation_x,vLocation_y,'g','LineWidth',4);
plot(4.57-(0:0.46:0.46),[3.05,3.05],'b','LineWidth',5);
flagPt = max(find([predictions_KF.Flag]))+1;
scatter(EKF_vLocation_x(flagPt),EKF_vLocation_y(flagPt),50,'+','black',LineWidth=20);
xlabel('horizontal position [meter]',FontSize=18);
ylabel('vertical position [meter]',FontSize=18);
legend({'real Projectile with drag','sensor Projectile measurements','KF estimation','basket','last measurement for prediction'},'FontSize',20);
axis([0 6 0 4.5]);
grid;
grid minor;
hold off;



% ordinary KF without drag
% figure()
% hold on;    
% plot(X_sampled(mask),Y_sampled(mask),'LineWidth',3);
% scatter(X_sampled_noised,Y_sampled_noised,40,'filled');
% title('Projectile along the XY plane (without drag) - standard KF - where the basket is located at (4.57,3.05)','FontSize',20);
% 
% predictions_KF = KalmanfilterAuxilary(Y_sampled_noised,X_sampled_noised,sParams);
% vLocation_x = [];
% vLocation_y = [];
% vVelocity_x = [];
% vVelocity_y = [];
% for ii = 1 : numel(predictions_KF)
%     vTmp = predictions_KF(ii).x;
%     vLocation_x = [vLocation_x; vTmp(1)];
%     vLocation_y = [vLocation_y; vTmp(2)];
%     vVelocity_x = [vVelocity_x; vTmp(3)];
%     vVelocity_y = [vVelocity_y; vTmp(4)];
% end 
% plot(vLocation_x,vLocation_y,'g','LineWidth',3);
% plot(4.57-(0:0.46:0.46),[3.05,3.05],'b','LineWidth',7);
% flagPt = max(find([predictions_KF.Flag]))+1;
% scatter(vLocation_x(flagPt),vLocation_y(flagPt),'filled','g');
% legend({'real Projectile without drag','sensor Projectile measurements','KF estimation','basket'},'FontSize',20);
% axis([0 6 0 4.5]);
% grid;
% grid minor;
% xlabel('horizontal position [meter]',FontSize=18);
% ylabel('vertical position [meter]',FontSize=18);
% hold off;



% %% Projectile estimation without drag
% Y_sampled = Y_wo(indicesT); % projectile sampling
% Y_sampled_noised = Y_sampled + normrnd(mu, sigma_y,size(indicesT));
% X_sampled = X_wo(indicesT);
% X_sampled_noised = X_sampled + normrnd(mu, sigma_x,size(indicesT));
% mask = (Y_sampled_noised >= 0);
% Y_sampled_noised = Y_sampled_noised(mask);
% X_sampled_noised = X_sampled_noised(mask);
% 
% 
% % Graphs
% figure()
% hold on;
% plot(X_sampled(mask),Y_sampled(mask));
% % plot(X_sampled,Y_sampled);
% plot(X_sampled_noised,Y_sampled_noised);
% title('Projectile along the XY plane (without drag)- where the basket is located at (7.24,3.05)');
% 
% predictions = KalmanfilterAuxilary(Y_sampled_noised,X_sampled_noised,dt_C);
% vLocation_x = [];
% vLocation_y = [];
% vVelocity_x = [];
% vVelocity_y = [];
% for ii = 1 : numel(predictions)
%     vTmp = predictions(ii).x;
%     vLocation_x = [vLocation_x; vTmp(1)];
%     vLocation_y = [vLocation_y; vTmp(2)];
%     vVelocity_x = [vVelocity_x; vTmp(3)];
%     vVelocity_y = [vVelocity_y; vTmp(4)];
% end 
% plot(vLocation_x,vLocation_y);
% plot(7.24-(0:0.46:0.46),[3.05,3.05]);
% flagPt = max(find([predictions.Flag]))+1;
% scatter(vLocation_x(flagPt),vLocation_y(flagPt),'filled','g');
% legend('real Projectile wo drag','sensor Projectile measurements','KF estimation');
% hold off;
% 
% 
% 
% % ------ Velocity Graphs ------
% % figure();
% % subplot(1,2,1);
% % hold on;
% % plot([1:length(vVelocity_y)],Vy(round(linspace(1,length(Vy),length(vVelocity_y)))));
% % plot(vVelocity_y);
% % hold off
% % title('Vy')
% % subplot(1,2,2);
% % hold on;
% % plot([1:length(vVelocity_y)],Vx(round(linspace(1,length(Vx),length(vVelocity_x)))));
% % plot(vVelocity_x);
% % hold off
% % title('Vx')
% 
% 
% 
% 
% 
% 
% % MaxDiviation = max(sqrt((Y_sampled(mask) - Y_sampled_noised).^2 +(X_sampled(mask) - X_sampled_noised).^2));
% % MSE = sqrt(sum((Y_sampled(mask) - Y_sampled_noised).^2 +(X_sampled(mask) - X_sampled_noised).^2));
% 
% 
% 
%%




