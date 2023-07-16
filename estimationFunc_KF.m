function [Est] = estimationFunc_KF(current_Samp,current_P,sParams)

BasketLocation = 4.57-(0:0.46:0.46);
BasketHeight = 3.05;
len = size(current_Samp,1);
x = {};
P = {};
Est = {};
for n = 10 : len % first 10 samples are to noisy, thus worthless 
    sParams.BallInitial_Y = current_Samp(n,2);
    sParams.BallInitial_X = current_Samp(n,1);
    sParams.v_x = current_Samp(n,3);
    sParams.v_y = current_Samp(n,4);
    sParams.Pcov = current_P(:,4*(n-2)+1:4*(n-2)+4);
    s = KF_Auxilary_MC(len-n,sParams);
    tmp = [s.x]';
    y = tmp(:,2);
    [M,I] = max(y);
    if M < BasketHeight
        continue
    end
    y = y(I:end);
    [B,J] = min(y(y >= BasketHeight)); % should we interpulate??
    [C,~] = max(y(y <= BasketHeight));
    if(abs(B - BasketHeight) <= abs(C - BasketHeight))
        idx = I + J -1;
    else 
        idx = I + J;
    end
    x{n} = s(idx).x;
    mu = x{n}(1);
    P{n} = s(idx).P;
    sigma = sqrt(P{n}(1,1));
  
    Est{n} = normcdf(BasketLocation(1),mu,sigma) - normcdf(BasketLocation(2),mu,sigma); 

end

end