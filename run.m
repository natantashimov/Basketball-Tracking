%%%%% Data acquisition 
intmat =   [911.346674479588	0	0
            0	911.689669739495	0
            960.280554981034	540.077792437830	1];
extmat = [1 0 0 0; 0 1 0 0; 0 0 1 0];
principalPoint = [960.280554981034, 540.077792437830];

% Depth in millimeters
depth = 6920;
% Z-coordinate is the depth
Z = depth;
C = [1700 280 -6920];
% xy = readtable('\locations.csv');
xy = readtable('.\locations\hit2.csv');
xy = table2array(xy);
x = xy(:,1) ; 
% x = x(find(x));
y = xy(:,2) ; 
% y = 1080-y;
% y = y(find(y));


X = zeros(size(x));
Y = zeros(size(y));
for i = 1 : length(x)
    % X-coordinate calculation
    X(i) = -(x(i) - principalPoint(1)) * Z / 911.346674479588; 
    % Y-coordinate calculation
    Y(i) = (y(i) - principalPoint(2)) * Z / 911.689669739495;
end

X = X + C(1);
Y = Y + C(2);
Z = Z + C(3);
h = 8197.5;

% Y = Y-h/2;
% Y = Y - min(Y);
% 

X = X(1:1*length(X));
Y = Y(1:1*length(Y));
X = X/1000;
Y = Y/1000;


%%%%% Initilization of params for the filter
% sParams.sampPrecet = 0.11;
sParams.R = 0.1205;
sParams.weight = 0.6;
sParams.BallInitial_Y = 1.5;
sParams.BallInitial_X = 0.15;
g = -9.813; % [m/sec^2], gravity constant
sParams.g = g;
dt_C = 1/240;
sParams.dt_C = dt_C;
sigma_x = 0.1; % [m], mean of daviation noise in the x axis
sParams.sigma_x = sigma_x;
sigma_y = 0.1; % [m], mean of daviation noise in the y axis
sParams.sigma_y = sigma_y;
sParams.sigma_w_EKF = 20; % process noise std
sParams.Pcov = 400; % initial state noise variance
dragCoeff = 0.47; % for a sphere
sParams.dragCoeff = dragCoeff;
crossSection = pi*sParams.R^2; % for a sphere
sParams.crossSection = crossSection;
rho = 1.225; % [kg/m^3] mass density of air
sParams.rho = rho;


XY = [X,Y];
XY_dup = XY;
count = 0;

for i = 2:length(XY)
    if XY_dup(i,:) == XY(i-1,:)
        XY_dup(i,:) = [-inf,-inf];
    end
end

for i = 1:length(XY)
    count = count + 1;
    if XY_dup(i,2) > 1.8 ||  XY_dup(i,1) > 2
        break
    end
end


% [~,index,~] = unique(XY,"rows");
X = XY_dup(count:end,1);
Y = XY_dup(count:end,2);

%%%%% Extraordiner EKF
sParams.sampPrecet = 0.1;
predictions_EKF = EKF_Auxilary(Y,X,sParams);
EKF_vLocation_x = [];
EKF_vLocation_y = [];
EKF_vVelocity_x = [];
EKF_vVelocity_y = [];
beta = [];
for ii = 1 : numel(predictions_EKF)
    vTmp = predictions_EKF(ii).x;
    beta = [beta; vTmp(5)];
    EKF_vLocation_x = [EKF_vLocation_x; vTmp(1)];
    EKF_vLocation_y = [EKF_vLocation_y; vTmp(2)];
    EKF_vVelocity_x = [EKF_vVelocity_x; vTmp(3)];
    EKF_vVelocity_y = [EKF_vVelocity_y; vTmp(4)];
end
EKF = [EKF_vLocation_x,EKF_vLocation_y];

%%%%% plotting
figure();
hold on;
% plot(X,Y,'b*');
% scatter(X(1:sParams.sampPrecet*length(X)),Y(1:sParams.sampPrecet*length(Y)),'g*');
plot(EKF_vVelocity_x);
plot(EKF_vVelocity_y);
% 
% hold off;
% legend("projectile","filter samples","KF","EKf");
% text(0.5, 0.5, "sampPrecet = " + sParams.sampPrecet+  " sigma_w_EKF = "+ sParams.sigma_w_EKF)
% T = table(EKF(2:end,1),EKF(2:end,2));
% filename = 'locations1.csv';
% writetable(T,filename);
% 
% T2 = table(KF(2:end,1),KF(2:end,2));
% filename = 'locations2.csv';
% writetable(T2,filename);
% 
figure
hold on;
plot(EKF_vLocation_x,EKF_vLocation_y);
ylim([0,6]);
BasketLocation = 4.57-(0:0.46:0.46);
BasketHight = 3.05*ones(size(BasketLocation));

scatter(X,Y,LineWidth=2);
scatter(X(1:sParams.sampPrecet*length(X)),Y(1:sParams.sampPrecet*length(Y)),'g*');
plot(BasketLocation,BasketHight,LineWidth=4);
ax = gca;
ax.XDir = 'reverse';

figure
plot(beta);
