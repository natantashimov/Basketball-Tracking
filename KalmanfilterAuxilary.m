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
for t = 1 : length(samples_Y)
   if t >= sParams.sampPrecet*length(samples_Y)
       samplesFlag = 0; 
   end
   s(end).Flag = samplesFlag;
   s(end).z = [samples_X(t);samples_Y(t)];  % create a measurement
   s(end+1) = kalmanf(s(end),samplesFlag); % perform a Kalman filter iteration
end
end
