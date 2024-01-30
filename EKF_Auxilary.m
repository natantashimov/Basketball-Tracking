%% Extended Kalman-filter function
% s.x = state vector at time k estimation.
% s.u = system control input, in our case it's the gravity constet g.
% s.z = vector of observation. 
% s.A = state transition matrix.
% s.P = state vector estimation covariance.
% s.Q = process noise covariance.
% s.R = measurement noise covariance.
% s.H = observation matrix.

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

s.x = [d        % x
       h        % y
       5        % x_dot
       8        % y_dot    
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
      0,        0,        2e-4];% needed small factor % 5x2 matrix
 
  
% % Define a process noise (std)
s.Q = sigma_w^2*s.B*s.B';  % variance, hence std^2
% % Define the measurement equation (sampeling aquisition matrix) 
s.H =  [1, 0, 0, 0, 0
        0, 1, 0, 0, 0];  %  [1 0]; % position only measurement. 
% % Define a measurement error (std) 
s.R = [sigma_x^2, 0
        0,        sigma_y^2]; % variance, hence std^2     
    
s.P = sParams.Pcov*eye(5);
s.P(1,1)=0.5;%2
s.P(2,2)=1;%2
s.P(3,3)=5;%10
s.P(4,4)=5;%10
s.P(5,5)=5;%25
    
samplesFlag = 1;
s.Flag = [];
for t = 1 : length(samples_Y)+200
%     if t >= 100
%         s(end).R = s(end).R*10e6;
%     end
   if t >= sParams.sampPrecet*length(samples_Y)
       samplesFlag = 0; 
       samples_X(t) = 0;
       samples_Y(t) = 0;
   end
   if ((samples_X(t) == -inf || samples_X(t) == inf) && (samples_Y(t) == -inf || samples_Y(t) == inf)) 
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
