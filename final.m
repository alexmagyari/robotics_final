
% Constants
comparison_threshold = .035;
comparison_counter_max = 5; % 5
width_cm = 27.81; %Width of cropped frame measured in cm
height_cm = 25; %Height of cropped frame measured in cm
height_offset = 8.25; % how far the base frame is out of the image in cm
d1 = 8.9; % cm
d2_true = 18; % cm
r2 = 5.25; % cm
d2 = sqrt(d2_true^2 + r2^2);
z_offset = -1; %cm

% a = arduino;
clc
clear('cam');
cam=webcam('USB  Live camera');
RGB = snapshot(cam);
failCount = 0;
[rows, columns, numberOfColorChannels] = size(RGB);
centerX = columns / 2;
centerY = rows / 2;
% Top left of image is 0,0
minBoundX = floor(centerX - (.3 * columns));
maxBoundX = floor(centerX + (.3 * columns));
minBoundY = floor(centerY - (.45 * rows));
maxBoundY = floor(centerY + (.35 * rows));

x = [[minBoundX minBoundX] [maxBoundX maxBoundX] [0 1] [0 1]];
y = [[0 1] [0 1] [minBoundY minBoundY] [maxBoundY maxBoundY]];

[rows, columns, numberOfColorChannels] = size(imcrop(RGB,[minBoundX minBoundY (maxBoundX - minBoundX) (maxBoundY - minBoundY)]));
centerX = columns / 2;

% Connect to Arduino
arduino_serial = serialport("COM3",9600);
arduino_serial.Timeout = 1;
configureTerminator(arduino_serial,"CR");


% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.000;
channel1Max = 255.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 255.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 163.000;
channel3Max = 199.000;

ball_location_cur = [0 0 z_offset];
ball_location_new = [0 0 z_offset];
ball_location_old = [0 0 z_offset];
motor_angles = [0 0 0];

waitForGo = false;
checkBallLocation = false;

failCount = 0;
similarity_counter = 0;
wait_counter = 0;
while(true)
    % Calculate % difference in ball location
    difference = ball_location_old - ball_location_new;
    for i=1:length(ball_location_new)
        if (ball_location_new(i) == 0)
            ball_location_new(i) = .0001;
        end
    end
    difference = abs(difference) ./ ball_location_new;
    if all(difference < [comparison_threshold comparison_threshold comparison_threshold])
        similarity_counter = similarity_counter + 1;
    else
        similarity_counter = 0;
    end
    if (similarity_counter == comparison_counter_max)
        ball_location_cur = ball_location_new;
        L = sqrt(ball_location_cur(1)^2 + ball_location_cur(2)^2 + ball_location_cur(3)^2);
        % These two equations derived from the Law of Cosines
        t2 = (pi / 2) - acos( ( d1^2 + L^2 - d2^2) / (2 * d1 * L));
        t3 = pi - acos( (d1^2 + d2^2 - L^2) / (2 * d1 * d2))- atan2(r2, d2_true);
        
        
        m1 = atan2(ball_location_cur(1),ball_location_cur(2));
        m1 = round(rad2deg(m1));
        m2 = round(rad2deg(t2));
        m3 = round(rad2deg(t3));
        if (~waitForGo && ~checkBallLocation)
            if (isreal(m1) && isreal(m2) && isreal(m3))
                arduino_serial.Timeout = 4;
                disp("Sending angles")
                motor_angles = [m1 m2 m3];
                flush(arduino_serial);
                writeline(arduino_serial, strcat(string(m1), ";", string(m2), ";", string(m3)))
                waitForGo = true;
                while (~(isstring(readline(arduino_serial))))
                    disp("Sending angles again")
                    flush(arduino_serial);
                    writeline(arduino_serial, strcat(string(m1), ";", string(m2), ";", string(m3)))
                    waitForGo = true;
                end
                arduino_serial.Timeout = 1;
            else
                disp('Out of bounds: \n')
                disp(m1)
                disp(m2)
                disp(m3)
            end
        end
    end


    if (waitForGo)
        disp("Waiting for that pesky Arduino")
        arduinoOut = readline(arduino_serial);
        wait_counter = wait_counter + 1;
        if (isstring(arduinoOut) || wait_counter > 20)
            disp("Arduino response received")
            checkBallLocation = true;
            waitForGo = false;
            flush(arduino_serial);
            wait_counter = 0;
        end
    end


    ball_location_old = ball_location_new;
    hold off
    lateral = "1";
    horizontal = "1";
    
    RGB = snapshot(cam);

    RGB = imcrop(RGB,[minBoundX minBoundY (maxBoundX - minBoundX) (maxBoundY - minBoundY)]);
    
    
    
    figure(1);
    imshow(RGB) % original
    hold on 
    yline(.05 * rows, 'c--', 'LineWidth', 1)
    xline(.5 * columns, 'r--', 'LineWidth', 1)
    viscircles([centerX (.02 * rows) ] , 55,'Color','g');
    d = digits();
    digits(2);
    txt ={join(['Ball X: ',  string(vpa(sym(ball_location_cur(1)),2)), "cm"]), join(['Ball Y: ',  string(vpa(sym(ball_location_cur(2)),2)), "cm"]), join(['Ball Z: ',  string(vpa(sym(ball_location_cur(3)),2)), "cm"]), join(["\theta1: ",  string(motor_angles(1)), char(176)]), join(["\theta2: ",  string(motor_angles(2)), char(176)]), join(["\theta3: ",  string(motor_angles(3)), char(176)])};
    rectangle('Position',[8 25 75 70],'FaceColor',[1 1 1])
    text(10, 60, txt)
    hold off
    digits(d);
    % From colorThresholder
    I = rgb2ycbcr(RGB);
    sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BW = sliderBW;
    maskedRGBImage = RGB;
    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
    
    %figure(2);clf;
    %imshow(maskedRGBImage)

    %https://www.mathworks.com/help/images/ref/bwconncomp.html is easier.
    CC = bwconncomp(sliderBW);
    s = regionprops(CC,'Centroid','Area');
    centroids = cat(1,s.Centroid);
    areas = cat(1,s.Area);
    [m,ind] = max(areas);
    figure(3);clf;
    imshow(double(sliderBW))

    if (isempty(centroids))
        ind = 1;
        centroids = [0 0; 0 0];
        areas = [0 0];
    end

    if (checkBallLocation)
        if (max(areas) < 300)

            failCount = 0;
            disp("Sending go.")
            writeline(arduino_serial, "1;");
            checkBallLocation = false;
        else
            if (failCount > 2)
                failCount = 0;
                disp("Sending retry.")
                disp(max(areas))
                writeline(arduino_serial, "0;");
                similarity_counter = 0;
                checkBallLocation = false;
            else
                failCount = failCount + 1
            end
        end
    else 
        if (~waitForGo)
            if ((max(areas) > 300) && all(centroids(ind,2) > (.1 * rows)))
                hold on
                    plot(centroids(ind,1),centroids(ind,2),'m*','markersize',32)
                hold off
    
                ball_location_new = [(((centroids(ind,1)/columns) * width_cm) - (width_cm / 2)) (((centroids(ind,2)/rows) * height_cm) + height_offset) ball_location_cur(3)];
            else
                disp("No objects found. Did the ball bounce out of the arena?")
                similarity_counter = 0;
                continue
            end
        end
    end
end
clear arduino_serial