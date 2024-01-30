function Indicator = ShotIndicator(current_Proj)
BasketLocation = 4.57-(0:0.46:0.46);
BasketHeight = 3.05;
y = current_Proj(:,2);
x = current_Proj(:,1);
[~,maxHightIdx] = max(y);
y = y(maxHightIdx:end);
[~,I] = min(y(y >= BasketHeight));
[~,J] = max(y(y <= BasketHeight));
mPt = current_Proj(maxHightIdx+I-2 : maxHightIdx+I+1,:);
mPt_x = [ones(size(mPt(:,1))),mPt(:,1)]; 
mPt_y = mPt(:,2);
theta = inv(mPt_x' * mPt_x) * mPt_x' * mPt_y;
b1 = theta(1);
m1 = theta(2);
b2 = BasketHeight;
m2 = 0;
inter = [1 -m1;1 -m2] \ [b1;b2];

% checking if the ball gets in, w.r.t the thickness of the ring 2 cm and
% half radius of 6 cm, (beacuse their is an angle)
if (inter(2) < BasketLocation(2)+0.02+0.06 | inter(2) > BasketLocation(1)-0.02-0.06)
% if (inter(2) < BasketLocation(2) | inter(2) > BasketLocation(1))
    Indicator = 0;
else
    Indicator = 1;
end
end