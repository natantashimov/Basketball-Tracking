function [v0,Angle,R,Weight] =  ParametersInitialization(dist_flag)
   
% Throw Params
if strcmp(dist_flag,'3Pt')
    BallMaxSpeed = 13; % [m/sec]
    BallMinSpeed = 12.4; % [m/sec]
    BallSpeedRange = BallMaxSpeed - BallMinSpeed;
    v0 = BallSpeedRange * rand + BallMinSpeed;
    
    MaxAngle =47; % [°]
    MinAngle = 45; % [°]
    AngleRange = MaxAngle - MinAngle;
    Angle = AngleRange * rand + MinAngle;
    
else
    BallMaxSpeed = 7.6; % [m/sec]
    BallMinSpeed = 7.4; % [m/sec]
    BallSpeedRange = BallMaxSpeed - BallMinSpeed;
    v0 = BallSpeedRange * rand + BallMinSpeed;
    
    MaxAngle = 62; % [°]
    MinAngle = 58; % [°]
    AngleRange = MaxAngle - MinAngle;
    Angle = AngleRange * rand + MinAngle;
end

% Ball Params
BallMaxRadius = 0.121; % [m]
BallMinRadius = 0.12; % [m]
BallRadiusRange = BallMaxRadius - BallMinRadius;
R = BallRadiusRange * rand + BallMinRadius;

MaxWeight = 0.53 ;% [Kg]
MinWeight = 0.49; % [Kg]
WeightRange = MaxWeight - MinWeight;
Weight = WeightRange * rand + MinWeight;

end
