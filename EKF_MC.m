function s = EKF_MC(s,alpha,dt)

 % local input assignment
 x = s.x;
 u = s.u;
 P = s.P;
 A = s.A;
 B = s.B;
 H = s.H;
 Q = s.Q;
 R = s.R;
 
 % KF algorithem
 x = time_update_function(x,u,alpha,dt) ; % Time Update (“Predict”)
 P = A * P * A' + Q;
 % output assignment
 s.x = x;
 s.P = P;
 
end


