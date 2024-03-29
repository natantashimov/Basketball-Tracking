clear all; close all; clc;
rng(1);
Projetile =  {}; 
sampled_Projetile = {};
predictions_KF = {};  predictions_EKF = {};
tmp1 = [];  tmp2 = [];  tmp3 = [];
MSE_KF_Pos = 0;  MSE_EKF_Pos = 0;  MSE_RAW_SAMPLES_Pos = 0; 
MSE_KF_Vel = 0;  MSE_EKF_Vel = 0;
Indicator = [];
Beta_vec = 0;
sigma_x_vec = 0;
sigma_y_vec = 0;
sigma_Vx_vec = 0;
sigma_Vy_vec = 0;

sampPrecet = 1;
numIter = 1;
for ii = 1 : numIter
    [Projetile{ii},sampled_Projetile{ii},maxidx,predictions_EKF{ii},predictions_KF{ii},sParams] = simFunc(sampPrecet);
    % Position
    current_Position = Projetile{ii}.Pos;
    current_sampled_Proj = sampled_Projetile{ii};
    current_KF_x = [predictions_KF{ii}.x]';
    current_EKF_x = [predictions_EKF{ii}.x]';
    current_KF_Position = current_KF_x(2:end,1:2);
    current_EKF_Position = current_EKF_x(2:end,1:2);
    tmp1 = (current_Position - current_KF_Position).^2; tmp1 = tmp1(:,1)+ tmp1(:,2);
    tmp2 = (current_Position - current_EKF_Position).^2; tmp2 = tmp2(:,1)+ tmp2(:,2);
    tmp3 = (current_Position - current_sampled_Proj).^2; tmp3 = tmp3(:,1)+ tmp3(:,2);
    MSE_KF_Pos = MSE_KF_Pos + tmp1(1:80);
    MSE_EKF_Pos = MSE_EKF_Pos + tmp2(1:80);
    MSE_RAW_SAMPLES_Pos = MSE_RAW_SAMPLES_Pos + tmp3(1:80);

    Indicator(ii) = ShotIndicator(current_Position);
    IndicatorEKF(ii) = ShotIndicator(current_EKF_Position);
    IndicatorKF(ii) = ShotIndicator(current_KF_Position);

    % Velocity
    current_Velocity = Projetile{ii}.Vel;
    current_KF_Velocity = current_KF_x(2:end,3:4);
    current_EKF_Velocity = current_EKF_x(2:end,3:4);
    tmp1 = (current_Velocity - current_KF_Velocity).^2; tmp1 = tmp1(:,1)+ tmp1(:,2);
    tmp2 = (current_Velocity - current_EKF_Velocity).^2; tmp2 = tmp2(:,1)+ tmp2(:,2);
    MSE_KF_Vel = MSE_KF_Vel + tmp1(1:80);
    MSE_EKF_Vel = MSE_EKF_Vel + tmp2(1:80);

    % Beta coefficiant
    Beta_tmp = current_EKF_x(2:end,end);
    Beta_vec = Beta_tmp(1:80) + Beta_vec;

    % covariance
    P_KF{ii} = [predictions_KF{ii}.P];
    P_EKF{ii} = [predictions_EKF{ii}.P]; % num of columns is 5*(length of kalman)
    % Position cov
    sigma_x_tmp = sqrt([P_EKF{ii}(1,1:5:400)]);
    sigma_x_vec = sigma_x_vec + sigma_x_tmp;
    sigma_y_tmp = sqrt([P_EKF{ii}(2,2:5:400)]);
    sigma_y_vec = sigma_y_vec + sigma_y_tmp;
    % velocity cov
    sigma_Vx_tmp = sqrt([P_EKF{ii}(3,3:5:400)]);
    sigma_Vx_vec = sigma_Vx_vec + sigma_Vx_tmp;
    sigma_Vy_tmp = sqrt([P_EKF{ii}(4,4:5:400)]);
    sigma_Vy_vec = sigma_Vy_vec + sigma_Vy_tmp;

   
    % estimation
    [Est_EKF{ii},f] = estimationFunc_EKF(current_EKF_x(1:80,1:4),P_EKF{ii}(:,1:400),sParams,current_Position);
    %%%%%%%%%%%%%%%%%%%
    [Est_KF{ii}] = estimationFunc_KF(current_KF_x(1:80,:),P_KF{ii}(:,1:320),sParams);
    %%%%%%%%%%%%%%%%%%%

    %    figure(ii);
    %    hold on;
    %    plot(current_Position(:,1),current_Position(:,2));
    %    plot(current_sampled_Proj(:,1),current_sampled_Proj(:,2));
    %    plot(current_EKF_x(:,1),current_EKF_x(:,2));
    %    BasketLocation = 4.57-(0:0.46:0.46);
    %    BasketHight = 3.05*ones(size(BasketLocation));
    %    plot(BasketLocation,BasketHight);
    %    hold off;
    %    legend('current_Position','current_sampled_Proj','current_EKF_x');

end

%%
avrage_EKF = zeros(1,45);
for jj = 0 : 44
    for ii = 1 : numIter
        avrage_EKF(end-jj) = avrage_EKF(end-jj) + Est_EKF{ii}{end-jj};
    end
end
avrage_EKF = avrage_EKF/numIter;

avrage_KF = zeros(1,45);
for jj = 0 : 44
    for ii = 1 : numIter
        avrage_KF(end-jj) = avrage_KF(end-jj) + Est_KF{ii}{end-jj};
    end
end
avrage_KF = avrage_KF/numIter;


avg_shot = mean(Indicator);
avg_EKF_pred = mean(IndicatorEKF);
avg_KF_pred = mean(IndicatorKF);
MissClassificationRate1 = mean(abs(Indicator - IndicatorEKF));
MissClassificationRate2 = mean(abs(Indicator - IndicatorKF));

% figure();
% title('shot probability w.r.t initial prediction sample',FontSize=20);
% hold on;
% plot(avg_shot*ones(1,45),LineWidth=2);
% plot(avrage_EKF,LineWidth=2);
% plot(avrage_KF,LineWidth=2);
% ylim([0,1]);
% legend({'player precentage','EKF predicted precentage','KF predicted precentage'},'FontSize',14);
% xlabel('sample [#]','FontSize',15);
% ylabel('Precentage [m]','FontSize',15);

% Position
RMSE_KF_Pos = sqrt(MSE_KF_Pos/numIter);
RMSE_EKF_Pos = sqrt(MSE_EKF_Pos/numIter);
RMSE_RAW_SAMPLES_Pos = sqrt(MSE_RAW_SAMPLES_Pos/numIter);

% Velocity
RMSE_KF_Vel = sqrt(MSE_KF_Vel/numIter);
RMSE_EKF_Vel = sqrt(MSE_EKF_Vel/numIter);

% Beta coefficiant
Beta = Beta_vec/numIter;

% Position Sigma
sigma_x = sigma_x_vec/numIter;
sigma_y = sigma_y_vec/numIter;
cov_XY = (sigma_x.^2 + sigma_y.^2);

% Velocity Sigma
sigma_Vx = sigma_Vx_vec/numIter;
sigma_Vy = sigma_Vy_vec/numIter;
cov_VxVy = (sigma_Vx.^2 + sigma_Vy.^2);


%%
%%%%% figures
% Positions error figure
%   y = (0:0.001:0.5);
%   I = maxidx*ones(size(y));
%
% figure();
% title('RMSE of Position Estimation',FontSize=30);
% hold on;
% x = (1:length(RMSE_KF_Pos)-4)*sParams.dt_C;
% plot(x,RMSE_KF_Pos(5:end),'r','LineWidth',4);
% plot(x,RMSE_EKF_Pos(5:end),'b','LineWidth',4);
% plot(x,RMSE_RAW_SAMPLES_Pos(5:end),'g-','LineWidth',4);
% plot(x,sqrt(cov_XY(5:end)),'k','LineWidth',4);
% %plot(I,y,'m-','LineWidth',1);
% legend({'RMSE KF','RMSE EKF','RMSE RAW SAMPLES','sqrt of cov of X&Y','max highet idx'},'FontSize',20);
% xlabel('time [sec]','FontSize',20);
% ylabel('Position std [m]','FontSize',20);
% ylim([0,0.1]);
% %xlim([5,80]);
% grid;
% grid minor;
% hold off;
%
%   % Velocity error figure
% figure();
% title('RMSE of Velocity Estimation',FontSize=30);
% hold on;
% plot(x,RMSE_KF_Vel(5:end),'r','LineWidth',4);
% plot(x,RMSE_EKF_Vel(5:end),'b','LineWidth',4);
% plot(x,sqrt(cov_VxVy(5:end)),'k','LineWidth',4);
% %plot(I,y,'m-');
% legend({'RMSE KF Velocity','RMSE EKF Velocity','sqrt of cov of Vx&Vy','max highet idx'},'FontSize',20);
% xlabel('time [sec]','FontSize',20);
% ylabel('Velocity std [m/s]','FontSize',20);
% ylim([0,1]);
% % xlim([5,80]);
% grid;
% grid minor;
% hold off;
%
%   % beta coefficiant error figure
% figure();
% title('Beta w.r.t time',FontSize=20);
% hold on;
% x1 = (1:length(Beta))*sParams.dt_C;
% plot(x1,Beta,'r','LineWidth',2);
% legend('Beta','FontSize',18);
% xlabel('time [sec]','FontSize',15);
% ylabel('coefficient [DL]','FontSize',15);
% ylim([0,2]);
% grid;
% grid minor;
% hold off;
%
%   % Position sigma figure
% figure();
% title('Position STD w.r.t time',FontSize=20);
% hold on;
% plot(x,sigma_x(5:end),'LineWidth',2);
% plot(x,sigma_y(5:end),'LineWidth',2);
% plot(x,sqrt(cov_XY(5:end)),'k','LineWidth',2);
% legend({'Position Sigma x','Position Sigma y','sqrt of cov of X&Y'},FontSize=15);
% xlabel('time [sec]','FontSize',15);
% ylabel('Position std [m]','FontSize',15);
% ylim([0,0.08]);
% % xlim([5,80]);
% grid;
% grid minor;
% hold off;
%
%   % Velocity sigma figure
% figure();
% title('Velocity STD w.r.t time',FontSize=20);
% hold on;
% plot(x,sigma_Vx(5:end),'LineWidth',2);
% plot(x,sigma_Vy(5:end),'LineWidth',2);
% plot(x,sqrt(cov_VxVy(5:end)),'k','LineWidth',2);
% legend({'Sigma Vx','Sigma Vy','sqrt of cov of Vx&Vy'},FontSize=15);
% xlabel('time [sec]');
% ylabel('Velocity std [m/s]');
% ylim([0,0.5]);
% % xlim([5,80]);
% grid;
% grid minor;
% hold off;



