% Basketball Trajectory Tracking and Estimation Script, using real world
% locations

% Define Constants:
BasketLocation = 4.57-(0:0.46:0.46);
BasketHeight = 3.05*ones(size(BasketLocation));

% Intrinsic and Extrinsic Parameters:
principalPoint = [960.280554981034, 540.077792437830];
depth = 6920;
Z = depth; 
C = [1370 280 -Z];

% Load Ball Trajectory Data (image coordinates):
xy = readtable(".\locations\locations_3pt_1.csv");
% xy = readtable(".\locations\locations_3pt_2.csv");
% xy = readtable(".\locations\locations_3pt_3.csv");
% xy = readtable(".\locations\locations_3pt_4.csv");
% xy = readtable(".\locations\locations_3pt_6.csv");
% xy = readtable(".\locations\locations_3pt_8.csv");


% Perform Convertions to get Real World Locations:
xy = table2array(xy);
x = xy(:,1) ; 
y = xy(:,2) ; 

X = zeros(size(x));
Y = zeros(size(y));
for i = 1 : length(x)
    % X-coordinate calculation
    X(i) = -(x(i) - principalPoint(1)) * Z / 911.346674479588; 
    % Y-coordinate calculation
    Y(i) = (y(i) - principalPoint(2)) * Z / 911.689669739495;
end

X = X + C(1);
Y = Y + C(2);
X = X(1:1*length(X));
Y = Y(1:1*length(Y));
X = X/1000;
Y = Y/1000;

%% until here we got locations in the real world coardinates

XY = [X,Y];
XY_dup = XY;
count = 1;

% next section meant to truncate all measures before release of the ball
for i = 2:length(XY)
    if XY_dup(i,:) == XY(i-1,:)
        XY_dup(i,:) = [-inf,-inf];
    end
end

X = XY_dup(count:end,1);
Y = XY_dup(count:end,2);

last = basket_ball_intersection(XY_dup', BasketHeight);


% Filter out data points outside the specified range and truncation conditions
data =[X,Y]; 
data = data(1:last, :);
first_idx = find(data(:,1) ~= inf & data(:,1) ~= -inf);
first_idx = first_idx(1);
data = data(first_idx:end,:);
TOT = last-first_idx+1;

% Initialize Parameters for Filtering:
% sParams.sampPrecet = 0.11;
sParams.R = 0.1205;
sParams.weight = 0.6;
sParams.BallInitial_Y = 1.5;
sParams.BallInitial_X = -1.6;%0.15;
g = -9.813; % [m/sec^2], gravity constant
sParams.g = g;
dt_C = 1/240;
sParams.dt_C = dt_C;
sigma_x = 0.1; % [m], mean of daviation noise in the x axis
sParams.sigma_x = sigma_x;
sigma_y = 0.1; % [m], mean of daviation noise in the y axis
sParams.sigma_y = sigma_y;
sigma_w_EKF = 5;
sParams.sigma_w_EKF = sigma_w_EKF; % process noise std
sParams.sigma_w_KF = 5;
sParams.Pcov = 400; % initial state noise variance
dragCoeff = 0.47; % for a sphere
sParams.dragCoeff = dragCoeff;
crossSection = pi*sParams.R^2; % for a sphere
sParams.crossSection = crossSection;
rho = 1.225; % [kg/m^3] mass density of air
sParams.rho = rho;

%% Initialize Video Reader and Writer:
obj = VideoReader('.\locations\3pt_1.avi');
% obj = VideoReader('.\locations\3pt_2.avi');
% obj = VideoReader('.\locations\3pt_3.avi');
% obj = VideoReader('.\locations\3pt_4.avi');
% obj = VideoReader('.\locations\3pt_6.avi');
% obj = VideoReader('.\locations\3pt_8.avi');

vid = read(obj);
frames = obj.NumFrames;
video_width = obj.Width;
video_height = obj.Height;
elipse_video = VideoWriter('ellipse.avi');
elipse_video.FrameRate = 20;
open(elipse_video);

% Initialize Figure:
fig = figure('Visible', 'off'); % Create an invisible figure
set(fig, 'Position', [0, 0, video_width, video_height]);
ax = axes('Parent', fig);
set(ax, 'Position', [0, 0, 1, 1]); % Set axes position to fill the figure
xlim([1, video_width]);
ylim([1, video_height]);

% Main Loop for Frame Processing:
for frame_idx = first_idx : last+50
    disp(['Processing Frame: ' num2str(frame_idx)]);  % Print frame_idx for debugging

    % Read the frame
    frame = read(obj, frame_idx);

    if frame_idx < first_idx+50
        writeVideo(elipse_video, frame);
        continue
    end

    % Perform Kalman Filtering and Trajectory Prediction
    if frame_idx <= last
        sParams.sampPrecet = max((frame_idx-first_idx)/TOT,10/TOT);
        predictions_EKF2 = EKF_Auxilary(data(:,2),data(:,1),sParams); %KalmanfilterAuxilary/EKF_Auxilary
        state_vector = [predictions_EKF2.x];
        pos = state_vector(1:2,:);
        vel = state_vector(3:4,:);
        P_EKF = [predictions_EKF2.P];

        x_ellipse = pos(1,:)*1000;
        x_ellipse = x_ellipse - C(1);
        x_ellipse = -x_ellipse;
        x_ellipse = x_ellipse*911.346674479588/6920 + principalPoint(1);

        y_ellipse = pos(2,:)*1000;
        y_ellipse = y_ellipse - C(2);
        y_ellipse = y_ellipse*911.689669739495/6920 + principalPoint(2);
        y_ellipse = 1080 - y_ellipse;

    end
    vfact = [(911.346674479588);(911.689669739495)]*(1000/6920);
    idx = basket_ball_intersection(pos, BasketHeight);

    % Display the frame on the entire figure
    imshow(frame, 'Parent', ax);

    % Plot Ellipse and Additional Information on the Frame:
    hold on;
    for i = frame_idx-first_idx:length(x_ellipse) % this for run over future prediction for current frame
        sdwidth = 1;
        means = [x_ellipse(i),y_ellipse(i)]';
        K = (i-1)*5 + 1; % K = (i-1)*5 + 1; % depends KF/EKF
        cov = P_EKF(1:2, K:K+1);
        npts=50;
        % plot the gaussian fits
        tt=linspace(0,2*pi,npts)';
        x = cos(tt); y=sin(tt);
        ap = [x(:) y(:)]';
        [v,d]=eig(cov);
        d = sdwidth * sqrt(d); % convert variance to sdwidth*sd
        ball_radius = 0.12;
        d_x = d(1,1);
        d_y = d(2,2);
        d_x = max(d_x,ball_radius*0.75);
        d_y = max(d_y,ball_radius*0.75);
        d = [d_x 0
            0   d_y];
        d_img = d.*sqrt(vfact*vfact');
        bp = (v*d_img*ap) + repmat(means, 1, size(ap,2));
        if (mod(i,2) == 0)
            plot(bp(1,:), bp(2,:),'r');
        end


        if (i == idx)
            x_c = pos(1,idx);
            sigma= d_x;
            y_c = pos(2,idx);
            delta_y = y_c - BasketHeight(1)-0.12;
            delta_t = delta_y/abs(vel(2,idx));
            delta_x = delta_t*abs(vel(1,idx));
            mu = x_c + delta_x;
            h = linspace(0,10,1000);
            g = normpdf(h,mu,sigma);
            rim_radius = 0.015;
            Est = normcdf(BasketLocation(1)-rim_radius,mu,sigma) - normcdf(BasketLocation(2)+rim_radius,mu,sigma);
        end

    end
    plot(x_ellipse(2:frame_idx-first_idx),y_ellipse(2:frame_idx-first_idx),'r','LineWidth',1.5);
    plot(x_ellipse(frame_idx-first_idx:end),y_ellipse(frame_idx-first_idx:end),'b-','LineWidth',1);

    if (frame_idx >= idx+first_idx)
        used_frames = idx+first_idx;
    else
        used_frames = frame_idx;
    end

    p0 = rectangle('Position',[1470,100,360,480],'FaceColor','white','EdgeColor','black'); % [x, y, width, height]
    p1 = text(1505,120,sprintf('used frames = %d',used_frames),'color','blue','FontSize',16);
    if (Est <= 0.5)
        color = 'red';
    elseif (Est <= 0.75)
        color = '#D95319';
    elseif (Est <= 0.9)
        color = '#EDB120';
    else
        color = 'green';
    end
    p10 = rectangle('Position',[1500,180,310,190],'FaceColor','white','EdgeColor','black'); % [x, y, width, height]
    p11 = text(1500,165,sprintf('Telemetry'),'color','black','FontSize',18);

    p3 = text(1505,190,sprintf('origin is on the free throw line'),'color','black','FontSize',16);
    p4 = text(1505,230,sprintf('horizontal position = %.3f [m]',pos(1,frame_idx)),'color','black','FontSize',16);
    p5 = text(1505,270,sprintf('vertical position = %.3f [m]',pos(2,frame_idx)),'color','black','FontSize',16);
    p6 = text(1505,310,sprintf('horizontal velocity = %.3f [m/s]',vel(1,frame_idx)),'color','black','FontSize',16);
    p7 = text(1505,350,sprintf('vertical velocity = %.3f [m/s]',vel(2,frame_idx)),'color','black','FontSize',16);

    p12 = rectangle('Position',[1500,410,310,150],'FaceColor','white','EdgeColor','black'); % [x, y, width, height]
    p13 = text(1500,395,sprintf('Estimation params'),'color','black','FontSize',18);
    p8 = text(1505,420,sprintf('horizontal mean = %.3f [m/s]',mu),'color','black','FontSize',16);
    p9 = text(1505,460,sprintf('horizontal std = %.3f [m/s]',sigma),'color','black','FontSize',16);
    p2 = text(1505,520,sprintf('estimated Pr = %.3f',Est),'color',color,'FontSize',22);

    axis off;
    hold off;
    % Capture the plot as an image and close the figure
    F = getframe(fig);

    % Overlay the Plot on the Frame and Write to Video
    frame_with_plot = F.cdata;
    frame_with_plot_resized = imresize(frame_with_plot, [video_height, video_width]);
    writeVideo(elipse_video, frame_with_plot_resized);

    % Delete Plot Objects
    delete(p0);
    delete(p1);
    delete(p2);
    delete(p3);
    delete(p4);
    delete(p5);
    delete(p6);
    delete(p7);
    delete(p8);
    delete(p9);
    delete(p10);
    delete(p11);
    delete(p12);
    delete(p13);

end

% Close Video Writer and Figure:
close(fig);
close(elipse_video);


% Additional Functions:
function idx = basket_ball_intersection(xy_2, BasketHeight)
        y = xy_2(2,:);
        [M,I] = max(y);
        y = y(I:end);
        y = abs(y);
        [B,J] = min(y(y >= BasketHeight(1)+0.12)); 
        idx = I + J - 1;
end




