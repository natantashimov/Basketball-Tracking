function [Est,f] = estimationFunc_EKF(current_Samp,current_P,sParams,current_Proj)

BasketLocation = 4.57-(0:0.46:0.46);
BasketHeight = 3.05;
len = size(current_Samp,1);
x = {};
P = {};
Est = {};
d = linspace(0,10,1000);

writerObj = VideoWriter('test2.avi');
writerObj.FrameRate = 2;

open(writerObj);

figure();
subplot(1,2,1);
hold on;
title('Intial positon of the ball for the prediction',FontSize=8);
grid;
grid minor;
y1 = current_Proj(:,2);
x1 = current_Proj(:,1);
plot(x1,y1,'LineWidth',2);


subplot(1,2,2);
hold on;
title('Estimated position around basket height',FontSize=8);
grid;
grid minor;

for n = 10 : 60 % first sample "has" no velocity, so it's worthless 
    subplot(1,2,1);
    axis([0 5 0 5]);
    xlabel('horizontal position [meter]',FontSize=14);
    ylabel('vertical position [meter]',FontSize=14);
    p7 = plot(BasketLocation,BasketHeight*ones(size(BasketLocation)),'b-','LineWidth',5);
    p8 = scatter(current_Samp(n,1),current_Samp(n,2),25,'r','filled');
    p9 = text(1.5,1.1,sprintf('SAMPLE=%d',n));

    sParams.BallInitial_Y = current_Samp(n,2);
    sParams.BallInitial_X = current_Samp(n,1);
    sParams.v_x = current_Samp(n,3);
    sParams.v_y = current_Samp(n,4);
    sParams.Pcov = current_P(:,5*(n-2)+1:5*(n-2)+5);
    s = EKF_Auxilary_MC(len-n,sParams);
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
    % graphs for avi format video
    Basket_y = BasketHeight*ones(size(BasketLocation));
    Basket_x = BasketLocation;
    filter_x = current_Samp(n,1);
    filter_y = current_Samp(n,2);

    g = normpdf(d,mu,sigma);

    subplot(1,2,2);
    p1 = plot(d, g+3.05, 'k', 'LineWidth',0.1);
    xlabel('horizontal position [meter]',FontSize=14);
    ylabel('vertical position [meter]',FontSize=14);
    p2 = plot(Basket_x,Basket_y,'b-','LineWidth',5);

    p3 = scatter([mu-sigma,mu+sigma],Basket_y,'g','filled');
    p6 = plot([mu-sigma,mu+sigma],Basket_y,'g-',LineWidth=3);
    p5 = scatter(mu,Basket_y,'r','filled');
    axis([2.5 6 0 8]);

    p4 = text(3.6,1.75,sprintf('SAMPLE=%d',n));
    p10 = text(3,0.75,sprintf('ESTIMATED Pr=%.5f',Est{n}));

    f = getframe(gcf);
%     legend('gaussian','basket','1 STD from estimated Mean','estimated CI','Mean');


    writeVideo(writerObj,f);
    delete(p1);
    delete(p2);
    delete(p3);
    delete(p4);   
    delete(p5);
    delete(p6);
    delete(p7);
    delete(p9);
    delete(p10);

    subplot(1,2,1)
    delete(p8);
    p8 = scatter(current_Samp(n,1),current_Samp(n,2),20,'black','filled');
end
    subplot(1,2,2);
    p1 = plot(d, g+3.05, 'k', 'LineWidth',0.1);

    xlabel('horizontal position [meter]',FontSize=14);
    ylabel('vertical position [meter]',FontSize=14);
    p2 = plot(Basket_x,Basket_y,'b-','LineWidth',5);

    p3 = scatter([mu-sigma,mu+sigma],Basket_y,'g','filled');
    p4 = text(3.6,1.75,sprintf('SAMPLE=%d',n));
    p10 = text(3,0.75,sprintf('ESTIMATED Pr=%.5f',Est{n}));
    p6 = plot([mu-sigma,mu+sigma],Basket_y,'g--',LineWidth=4);
    p5 = scatter(mu,Basket_y,'r','filled');

    subplot(1,2,1)
    p7 = plot(BasketLocation,BasketHeight*ones(size(BasketLocation)),'b-','LineWidth',5);
    p8 = scatter(x1(n),y1(n),25,'r','filled');
    p9 = text(1.5,1.1,sprintf('SAMPLE=%d',n));
close(writerObj);

f = 0;
end