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


