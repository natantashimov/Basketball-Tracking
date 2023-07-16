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


