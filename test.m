intmat =   [911.346674479588	0	0
            0	911.689669739495	0
            960.280554981034	540.077792437830	1];

u0 = 960.280554981034;
v0 = 540.077792437830;

p = [-170; -28; 692; 1];
p_basket = [312; 259; 607; 1];
p_center_bot = [312; 263; 684.5; 1];
p_center_up = [312; 278; 684.5; 1];
extmat = [1 0 0 0; 0 1 0 0; 0 0 1 0];

res = intmat.'*extmat*p_center_up;

figure;
imshow("hit3.jpg");
impixelinfo;

res = res/res(3);
res(1) = 1920 - res(1);
res(2) = 1080 - res(2);

v = [res(1);res(2);1];
v = inv(intmat')*v*684.5; % real world


