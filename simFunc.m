function [Projetile,sampled_Projetile,maxidx_1,predictions_EKF,predictions_KF,sParams] = simFunc(sampPrecet)
%% Initilization
sParams.sampPrecet = sampPrecet;
[V0,Angle,R,Weight] =  ParametersInitialization('free_throw'); % 3Pt,free_throw
sParams.R = 0.1205;
sParams.weight = 0.6;

% Weight = 0.6; R = 0.1205;
% % end tmp
BallInitial_X = 0 + 0.1; %7.24 (4.57) % [m], distance from the basket to the 3-point (free-throw) line
BallInitial_Y = 1.91 + 0.5;  % [m]
sParams.BallInitial_Y = 1.8 + 0.4;
sParams.BallInitial_X = 0.1;

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

%% Projectiles
% projectile movment equetions
X0 = BallInitial_X; % basket location is (0,BasketHight)
Y0 = BallInitial_Y;
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
    vel_wo_all = [vel_wo_all;vel_wo_now];
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

[~,maxidx_1] = max(Y); maxidx_1 = maxidx_1/50;
[~,maxidx_2] = max(Y_wo);

%% Projectile estimation with drag
rng();
Y_sampled = Y(indicesT); % projectile sampling
Y_sampled_noised = Y_sampled + normrnd(mu,sigma_y,size(indicesT));
X_sampled = X(indicesT);
X_sampled_noised = X_sampled + normrnd(mu,sigma_x,size(indicesT));
mask = (Y_sampled_noised >= 0);
Y_sampled_noised = Y_sampled_noised(mask);
X_sampled_noised = X_sampled_noised(mask);
vel_all = vel_all(:,indicesT);
vel_all = vel_all(:,mask);

%%%%%%%%%%%%%%%%%%% OUTPUTS
Projetile.Pos = [X_sampled(mask);Y_sampled(mask)]';
Projetile.Vel = vel_all';
sampled_Projetile = [X_sampled_noised;Y_sampled_noised]';


%%%%%%%%%%%%%%%%%%% KALMAN Estimations

% Extraordiner EKF
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

% ordinary KF

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

end

