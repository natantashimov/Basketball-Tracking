% Path to your video file
video_path = './videos/airball.mp4';

% Define the size and circularity thresholds for filtering
min_ball_radius = 13;
max_ball_radius = 20;
circularity_threshold = 0.8;

% Create a VideoReader object
video_reader = VideoReader(video_path);

% Create a CSV file for storing the ball coordinates
csv_file = fopen('locations.csv', 'w');
fprintf(csv_file, 'Frame,X,Y\n');

frame_count = 0;

% Create a figure to display the video
figure('Name', 'Ball Detection', 'Position', [100, 100, 800, 600]);

% Read and process each frame of the video
while hasFrame(video_reader)
    frame = readFrame(video_reader);
    
    % Convert the frame to grayscale
    gray = rgb2gray(frame);
    
    % Apply Gaussian blur to reduce noise
    blurred = imgaussfilt(gray, 5);
    
    % Detect circles (balls) using Hough Circle Transform
    circles = imfindcircles(blurred, [min_ball_radius, max_ball_radius], ...
        'ObjectPolarity', 'bright', 'Sensitivity', 0.9);
    
    if ~isempty(circles)
        % Filter and verify the ball detections
        num_circles = size(circles, 1);
        for i = 1:num_circles
            x = circles(i, 1);
            y = circles(i, 2);
            radius = circles(i, 3);
            
            % Create a mask for the current circle
            mask = false(size(gray));
            [X, Y] = meshgrid(1:size(gray, 2), 1:size(gray, 1));
            mask(sqrt((X - x).^2 + (Y - y).^2) <= radius) = true;
            
            % Calculate circularity
            stats = regionprops(mask, 'Perimeter', 'Area');
            perimeter = stats.Perimeter;
            area = stats.Area;
            circularity = 4 * pi * area / (perimeter^2);
            
            % Check circularity
            if circularity < circularity_threshold
                continue;
            end
            
            % Ball detection passed filters, write the coordinates to the CSV file
            fprintf(csv_file, '%d,%d,%d\n', frame_count, round(x), round(y));
            
            % Draw a circle around the detected ball
            frame = insertShape(frame, 'Circle', [x, y, radius], 'LineWidth', 2, 'Color', 'r');
        end
    end
    
    % Display the frame with ball detections
    imshow(frame);
    title(sprintf('Frame %d', frame_count));
    
    frame_count = frame_count + 1;
    
    % Pause to show the frame (adjust the delay as needed)
    pause(0.02);
end

% Close the CSV file
fclose(csv_file);

% Display a message when processing is complete
disp('Ball detection completed and locations saved to "locations.csv"');
