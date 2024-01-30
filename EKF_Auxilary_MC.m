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
function s = EKF_Auxilary_MC(len,sParams)
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
v_x = sParams.v_x;
v_y = sParams.v_y;
% x = f_bal(x,u)
s.x = [d        % x
       h        % y
       v_x        % x_dot
       v_y        % y_dot    
       beta];  % drag correction factor coefficiant 
                % 5x1 matrix
    
sigma_x = sParams.sigma_x;
sigma_y = sParams.sigma_y; % [m], mean of daviation noise in the y axis
sigma_w = sParams.sigma_w_EKF;
g = sParams.g; % [m/sec^2], gravity constant
s.u = [0
       g];    % 2x1 vector
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
    
s.P = sParams.Pcov;

for t = 1 : len+200
   s(end+1) = EKF_MC(s(end),alpha,dt); % perform a Kalman filter iteration
   Vx = s(end).x(3);
   Vy = s(end).x(4);
   beta = s(end).x(5);
  
   % % % % acc = alpha*norm(v)*[Vx,Vy]+[0,g];
   V_norm = norm([Vx,Vy]);
   s(end).A(3,3) = 1 + alpha*beta*dt*(2*Vx^2 + Vy^2)/V_norm; % dvx/dvx
   s(end).A(3,4) = alpha*beta*dt*Vx*Vy/V_norm; % dvx/dvy 
   s(end).A(4,4) = 1 + alpha*beta*dt*(Vx^2+2*Vy^2)/V_norm; % dvy/dvy
   s(end).A(4,3) = alpha*beta*dt*Vx*Vy/V_norm; % dvy/dvx 
   s(end).A(4,5) = alpha*dt*V_norm*Vy;
   s(end).A(3,5) = alpha*dt*V_norm*Vx;
end

end
