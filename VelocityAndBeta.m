numFiles = 12;

% Define the range
range = 10;

% Initialize separate arrays
for i = 1:numFiles
    filename = sprintf('3pt_%d.xls', i);
    varName = sprintf('xy%d', i);
    
    % Read the data from the Excel file
    data = xlsread(filename); % Assuming the data is in numeric format
    
    % Create a logical index for points within the specified range
    rangeIndices = abs(data(:, 1)) <= range & abs(data(:, 2)) <= range;
    
    % Create a logical index for points where x <= 3 and y >= 3.3
    truncateIndices = data(:, 1) >= 3 & data(:, 2) <= 3.3;
    first = find(truncateIndices);
    first = first(1);
    
    % Combine the range and truncation indices
    validIndices = rangeIndices(1:first);   

    % Filter out data points outside the specified range and truncation conditions
    data = data(validIndices, :);
    
    % Save the filtered data as an array with the dynamic variable name
    eval([varName ' = data;']);

end

%%
BasketLocation = 4.57-(0:0.46:0.46);
BasketHeight = 3.05*ones(size(BasketLocation));
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
sigma_x = 0.05; % [m], mean of daviation noise in the x axis
sParams.sigma_x = sigma_x;
sigma_y = 0.05; % [m], mean of daviation noise in the y axis
sParams.sigma_y = sigma_y;
sigma_w_EKF = 10;
sParams.sigma_w_EKF = sigma_w_EKF; % process noise std
sParams.sigma_w_KF = 10;
sParams.Pcov = 400; % initial state noise variance
dragCoeff = 0.47; % for a sphere
sParams.dragCoeff = dragCoeff;
crossSection = pi*sParams.R^2; % for a sphere
sParams.crossSection = crossSection;
rho = 1.225; % [kg/m^3] mass density of air
sParams.rho = rho;

% avg = zeros(2,12);
vMSE = zeros(2,5,12);
for i = 1 : numFiles
    varName = sprintf('xy%d', i);
    data = eval(varName);
    sParams.sigma_w_EKF = 10;
    sParams.sampPrecet = 0.5;
%     predictions_EKF2 = EKF_Auxilary(data(:,2),data(:,1),sParams);
    predictions_EKF2 = KalmanfilterAuxilary(data(:,2),data(:,1),sParams);
    xy_2 = [predictions_EKF2.x]; 
    xy_2 = xy_2(1:2,:);
    P_EKF = [predictions_EKF2.P];

    y = xy_2(2,:);
    [M,I] = max(y);
    if M < BasketHeight(1)
        continue
    end
    y = y(I:end);
    [B,J] = min(y(y >= BasketHeight(1))); % should we interpulate??
    [C,~] = max(y(y <= BasketHeight(1)));
    if(abs(B - BasketHeight(1)) <= abs(C - BasketHeight(1)))
        idx = I + J -1;
    else 
        idx = I + J;
    end
    mu = xy_2(1,idx);
    %sigma = sqrt(P_EKF(1,(idx-1)*5+1));
    sigma = sqrt(P_EKF(1,(idx-1)*4+1));
    d = linspace(0,10,1000);

    g = normpdf(d,mu,sigma);

    Est = normcdf(BasketLocation(1),mu,sigma) - normcdf(BasketLocation(2),mu,sigma); 
    % graphs for avi format video
    Basket_y = BasketHeight(1)*ones(size(BasketLocation));
    Basket_x = BasketLocation;

    figure()
    hold on;
    plot(d, g+3.05, 'k', 'LineWidth',0.1);
    xlabel('horizontal position [meter]',FontSize=14);
    ylabel('vertical position [meter]',FontSize=14);
    plot(Basket_x,Basket_y,'b-','LineWidth',5);

%     scatter([mu-sigma,mu+sigma],Basket_y,'g','filled');
    plot([mu-sigma,mu+sigma],Basket_y,'g-',LineWidth=3);
    scatter(mu,Basket_y,'r','filled');
    axis([2.5 6 0 8]);

    MSE = mse_per_file(data,sParams,i);

end


function MSE = mse_per_file(data,sParams,i)
    sParams.sampPrecet = 1;
    tmp = sParams.sigma_w_EKF;
    sParams.sigma_w_EKF = 10;
    predictions_EKF1 = EKF_Auxilary(data(:,2),data(:,1),sParams);
    sParams.sigma_w_EKF = tmp;
    sParams.sampPrecet = 0.5;
%     predictions_EKF2 = EKF_Auxilary(data(:,2),data(:,1),sParams);
    predictions_EKF2 = KalmanfilterAuxilary(data(:,2),data(:,1),sParams);
%     MSE = ([predictions_EKF1.x] - [predictions_EKF2.x]).^2;
%     MSE = sqrt(sum(MSE(1:2,:),2)/(size(data,1)/2));
    MSE = 0;
    xy1 = [predictions_EKF1.x]; xy1 = xy1(1:2,:);
    xy2 = [predictions_EKF2.x]; xy2 = xy2(1:2,:);
    BasketLocation = 4.57-(0:0.46:0.46);
    BasketHight = 3.05*ones(size(BasketLocation));
    figure();
    plot(BasketLocation,BasketHight,xy1(1,:),xy1(2,:),xy2(1,:),xy2(2,:));
    title("w = " + sParams.sigma_w_EKF+ ", file number = " + i);
    legend("basket", "true", "estimation");

     tmp = [predictions_EKF2.x];
     tmpp = [predictions_EKF1.x];
     tmp2 = tmp(3:4,:);
     tmp1 = tmpp(3:4,:);
     figure(); 
     plot(tmp2(1,:))
     hold on;
     plot(tmp1(1,:));
     title("vx")

      figure(); 
      plot(tmp2(2,:));
      hold on;
      plot(tmp1(2,:)); 
      title("vy")

%       beta = tmp(5,:);
%       figure(); 
%       plot(beta)
%       title("beta")

end




%% Extended Kalman-filter function
% s.x = state vector at time k estimation.
% s.u = system control input, in our case it's the gravity constet g.
% s.z = vector of observation. 
% s.A = state transition matrix.
% s.P = state vector estimation covariance.
% s.Q = process noise covariance.
% s.R = measurement noise covariance.
% s.H = observation matrix.
%
% 
% 
% 
% 
% 
% 
function s = EKF_Auxilary(samples_Y,samples_X,sParams)
clear s
dragCoeff = sParams.dragCoeff; % for a sphere
R = sParams.R; % [m]
rho = sParams.rho; % [kg/m^3] mass density of air
weight = sParams.weight; % [kg] ball mass
crossSection = pi*R^2; % for a sphere
alpha = -dragCoeff*crossSection*rho/(2*weight); % drag extended coefficiant 
beta = 1;
h = sParams.BallInitial_Y;
d = sParams.BallInitial_X;
% x = f_bal(x,u)
s.x = [d        % x
       h        % y
       5        % x_dot
       5        % y_dot    
       beta];  % drag correction factor coefficiant 
                % 5x1 matrix
    
sigma_x = sParams.sigma_x;
sigma_y = sParams.sigma_y; % [m], mean of daviation noise in the y axis
sigma_w = sParams.sigma_w_EKF;
g = sParams.g; % [m/sec^2], gravity constant
s.u = [0
       g
       0];    % 2x1 vector
dt = sParams.dt_C;

% acc = alpha*norm(v)*[Vx,Vy]+[0,g];
s.A = [1 0 dt 0  0  
       0 1 0  dt 0  
       0 0 1  0  0  
       0 0 0  1  0  
       0 0 0  0  1]; 
                    % 5x5 matrix
   
s.B =[0.5*dt^2, 0,        0  
      0,        0.5*dt^2, 0 
      dt,       0,        0
      0,        dt,       0
      0,        0,        2e-4];% needed small factor      % 5x2 matrix
 
  
% % Define a process noise (std)
s.Q = sigma_w^2*s.B*s.B';  % variance, hence std^2
% % Define the measurement equation (sampeling aquisition matrix) 
s.H =  [1, 0, 0, 0, 0
        0, 1, 0, 0, 0];  %  [1 0]; % position only measurement. 
% % Define a measurement error (std) 
s.R = [sigma_x^2, 0
        0,        sigma_y^2]; % variance, hence std^2     
    
s.P = sParams.Pcov*eye(5);
s.P(1,1)=2;
s.P(2,2)=2;
s.P(3,3)=20;
s.P(4,4)=20;
s.P(5,5)=25;
    
samplesFlag = 1;
s.Flag = [];
for t = 1 : length(samples_Y)+100
%     if t >= 100
%         s(end).R = s(end).R*10e6;
%     end
   if t >= sParams.sampPrecet*length(samples_Y)
       samplesFlag = 0; 
       samples_X(t) = 0;
       samples_Y(t) = 0;
   end
   if (samples_X(t) == -inf && samples_Y(t) == -inf)
       samplesFlag = 0;
   end
   s(end).Flag = samplesFlag;
   s(end).z = [samples_X(t);samples_Y(t)];  % create a measurement
   s(end+1) = EKF(s(end),samplesFlag,alpha,dt); % perform a Kalman filter iteration
   Vx = s(end).x(3);
   Vy = s(end).x(4);
   beta = s(end).x(5);
   samplesFlag = 1;
   % % % % acc = alpha*norm(v)*[Vx,Vy]+[0,g];
   
   %%%%%%%%%%%%%%%%% new, mathematicly correct version
   V_norm = norm([Vx,Vy]);
   s(end).A(3,3) = 1 + alpha*beta*dt*(2*Vx^2 + Vy^2)/V_norm; % dvx/dvx
   s(end).A(3,4) = alpha*beta*dt*Vx*Vy/V_norm; % dvx/dvy 
   s(end).A(4,4) = 1 + alpha*beta*dt*(Vx^2+2*Vy^2)/V_norm; % dvy/dvy
   s(end).A(4,3) = alpha*beta*dt*Vx*Vy/V_norm; % dvy/dvx 
   s(end).A(4,5) = alpha*dt*V_norm*Vy;
   s(end).A(3,5) = alpha*dt*V_norm*Vx;


   %%%%%%%%%%%%%%%%%%% old version
%    s(end).A(3,3) = 1 + alpha*beta*dt*(2*Vx^2 + Vy^2)/(Vx^2 + Vy^2); % dvx/dvx
%    s(end).A(3,4) = alpha*beta*dt*Vx*Vy/(Vx^2 + Vy^2); % dvx/dvy 
%    s(end).A(4,4) = 1 + alpha*beta*dt*Vx*Vy/(Vx^2 + Vy^2); % dvy/dvy
%    s(end).A(4,3) = alpha*beta*dt*(2*Vx^2 + Vy^2)/(Vx^2 + Vy^2); % dvy/dvx
%    s(end).A(3,5) = alpha*dt*sqrt(Vx^2 + Vy^2)*Vx;
%    s(end).A(4,5) = alpha*dt*sqrt(Vx^2 + Vy^2)*Vy;
   %
end
end

function s = EKF(s,samplesFlag,alpha,dt)

 % local input assignment
 x = s.x;
 u = s.u;
 z = s.z;
 P = s.P;
 A = s.A;
 B = s.B;
 H = s.H;
 Q = s.Q;
 R = s.R;
 
 % KF algorithem
 x = time_update_function(x,u,alpha,dt) ; % Time Update (“Predict”)
 P = A * P * A' + Q;
 KG = P * H' /(H * P * H' + R); 
 if samplesFlag == 1
    x = x + KG * (z - H * x); % Measurement Update (“Correct”)
    P = P - KG * H * P; 
 end
 % output assignment
 s.x = x;
 s.P = P;
 
end


% x_time_update
function x = time_update_function(state_vec,input,alpha,dt) 
pos_now = state_vec(1:2);
vel_now = state_vec(3:4);
beta_now = state_vec(5);
g = input(2);
v = norm(vel_now);
acc_temp = beta_now*alpha*v*vel_now+[0;g];
vel_new = vel_now+acc_temp*dt;
pos_new = pos_now+(vel_new+vel_now)/2*dt;
pos_now = pos_new;
vel_now = vel_new;
beta_new = beta_now; 
x = [pos_new;vel_new;beta_new];

end


%% Kalman-filter function
% s.x = state vector at time k estimation.
% s.u = system control input, in our case it's the gravity constet g.
% s.z = vector of observation. 
% s.A = state transition matrix.
% s.P = state vector estimation covariance.
% s.Q = process noise covariance.
% s.R = measurement noise covariance.
% s.H = observation matrix.
%
% 
% 
% 
% 
% 
% 
function s = KalmanfilterAuxilary(samples_Y,samples_X,sParams)
clear s
h = sParams.BallInitial_Y;
d = sParams.BallInitial_X;
% s.x = s.A*s.x +s.B*s.u
s.x = [d        % x
       h        % y
       0        % x_dot
       0];      % y_dot    % 4x1 matrix

sigma_x = sParams.sigma_x;
sigma_y = sParams.sigma_y; % [m], mean of daviation noise in the y axis
sigma_w = sParams.sigma_w_KF;
g = sParams.g; % [m/sec^2], gravity constant
s.u = [0
       g];    % 2x1 vector
dt = sParams.dt_C;

s.A = [1 0 dt 0      
       0 1 0  dt
       0 0 1  0
       0 0 0  1];       
   
s.B =[0.5*dt^2, 0  
      0,        0.5*dt^2 
      dt,       0
      0,        dt ];      % 4x2 matrix
          
% % Define a process noise (std)
s.Q = sigma_w^2*s.B*s.B' ;  % variance, hence std^2
% % Define the measurement equation (sampeling aquisition matrix) 
s.H =  [1, 0, 0, 0
        0, 1, 0, 0];  %  [1 0]; % position only measurement. 
% % Define a measurement error (std) 
s.R = [sigma_x^2, 0
        0,        sigma_y^2]; % variance, hence std^2     
    
s.P = sParams.Pcov*eye(4);
s.P(1,1)=2;
s.P(2,2)=2;
s.P(3,3)=20;
s.P(4,4)=20;


samplesFlag = 1;
s.Flag = [];
for t = 1 : length(samples_Y) + 100
   if t >= sParams.sampPrecet*length(samples_Y)
       samplesFlag = 0; 
       samples_X(t) = 0;
       samples_Y(t) = 0;
   end
   if (samples_X(t) == -inf && samples_Y(t) == -inf)
       samplesFlag = 0;
   end
   s(end).Flag = samplesFlag;
   s(end).z = [samples_X(t);samples_Y(t)];  % create a measurement
   s(end+1) = kalmanf(s(end),samplesFlag); % perform a Kalman filter iteration
end
end


function s = kalmanf(s,samplesFlag)

 % local input assignment
 x = s.x;
 u = s.u;
 z = s.z;
 P = s.P;
 A = s.A;
 B = s.B;
 H = s.H;
 Q = s.Q;
 R = s.R;
 
 % KF algorithem
 x = A * x + B * u; % Time Update (“Predict”)
 P = A * P * A' + Q;
 KG = P * H' * inv(H * P * H' + R); 
 if samplesFlag == 1
    x = x + KG * (z - H * x); % Measurement Update (“Correct”)
    P = P - KG * H * P; 
 end
 % output assignment
 s.x = x;
 s.P = P;
 
end



