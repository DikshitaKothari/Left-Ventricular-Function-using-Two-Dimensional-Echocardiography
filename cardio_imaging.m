function[SV, ESV, EDV, EF, CO] = cardio_imaging(apical, apex, mitralvalve, pap)

% Pause time between frames is 0.1 seconds 
p = 0.01; 

% Choose the frames and measure the end systolic volume 
fprintf('The first measurement is done for the end systolic volume.\nMeasurements must be made for the end systolic volume only.\n');

% Pause to read dialogue
pause(2) 

% Frame number initiation
n = 1;              

% Create a VideoReader object
videoObj_apical4ch = VideoReader(apical);

% Read and display the video frames
while hasFrame(videoObj_apical4ch)
    videoFrame_apical4ch = readFrame(videoObj_apical4ch);
    imshow(videoFrame_apical4ch);
    title_str = sprintf('The frame number is %d.', n);
    title(title_str);    
    pause(p);
    n = n+1;        % Increment Frame Number 
end

% Frame to be read
frame_number = input('Enter the desired frame number: ');
fprintf('The frame selected is %d.', frame_number);

% Set the CurrentTime property to the desired frame
videoObj_apical4ch.CurrentTime = (frame_number - 1) / videoObj_apical4ch.FrameRate;

% Read the desired frame
frame = readFrame(videoObj_apical4ch);

% Display the frame
imshow(frame);
title_str = sprintf('The frame number is %d.', frame_number);
title(title_str);

% Select two points in the frame 
[x, y] = ginput(2);

% Define two points
point1 = [x(1), y(1)];
point2 = [x(2), y(2)];

% Calculate the Euclidean distance between the two points
L = sqrt(sum(power(point1 - point2, 2)));
h = L/3;

% Use scaling factor to get the distance value in cm
h = h/10;

% Print obtained value
fprintf('\nLong axis end systolic is %d. \n', h);


%%
% Frame number initiation
n = 1;           

% Create a VideoReader object
videoObj_shortaxis_apex = VideoReader(apex);

% Read and display the video frames
while hasFrame(videoObj_shortaxis_apex)
    videoFrame_shortaxis_apex = readFrame(videoObj_shortaxis_apex);
    imshow(videoFrame_shortaxis_apex);
    title_str = sprintf('The frame number is %d.', n);
    title(title_str);    
    pause(p);
    n = n+1;        % Increment Frame Number 
end

% Frame to be read
frame_number = input('\nEnter the desired frame number: ');
fprintf('\nThe frame selected is %d', frame_number);

% Set the CurrentTime property to the desired frame
videoObj_shortaxis_apex.CurrentTime = (frame_number - 1) / videoObj_shortaxis_apex.FrameRate;

% Read the desired frame
frame = readFrame(videoObj_shortaxis_apex);

% Display the frame
imshow(frame);
title_str = sprintf('The frame number is %d.', frame_number);
title(title_str);

% Select two points in the frame 
[x, y] = ginput(2);

% Define two points
point1 = [x(1), y(1)];
point2 = [x(2), y(2)];

% Calculate the Euclidean distance between the two points
shortaxis_apex = sqrt(sum(power(point1 - point2, 2)));

% Use scaling factor to get the distance value in cm
shortaxis_apex = shortaxis_apex/48;

% Print obtained value
fprintf('\nThe end systolic short axis apex value is %d', shortaxis_apex);

%%
% Frame number initiation
n = 1;      

% Create a VideoReader object
videoObj_shortaxis_mitralvalve = VideoReader(mitralvalve);

% Read and display the video frames
while hasFrame(videoObj_shortaxis_mitralvalve)
    videoFrame_shortaxis_mitralvalve = readFrame(videoObj_shortaxis_mitralvalve);
    imshow(videoFrame_shortaxis_mitralvalve);
    title_str = sprintf('The frame number is %d', n);
    title(title_str);    
    pause(p);
    n = n+1;        % Increment Frame Number 
end

% Frame to be read
frame_number = input('\nEnter the desired frame number: ');
fprintf('\nThe frame selected is %d', frame_number);

% Set the CurrentTime property to the desired frame
videoObj_shortaxis_mitralvalve.CurrentTime = (frame_number - 1) / videoObj_shortaxis_mitralvalve.FrameRate;

% Read the desired frame
frame = readFrame(videoObj_shortaxis_mitralvalve);

% Display the frame
imshow(frame);
title_str = sprintf('The frame number is %d', frame_number);
title(title_str);

% Select two points in the frame 
[x, y] = ginput(2);

% Define two points
point1 = [x(1), y(1)];
point2 = [x(2), y(2)];

% Calculate the Euclidean distance between the two points
shortaxis_mitralvalve = sqrt(sum(power(point1 - point2, 2)));

% Use scaling factor to get the distance value in cm
shortaxis_mitralvalve = shortaxis_mitralvalve/48;

% Print obtained value
fprintf('\nThe end systolic short axis mitral valve value is %d', shortaxis_mitralvalve);

%%
% Frame number initiation
n = 1;        

% Create a VideoReader object
videoObj_shortaxis_pap = VideoReader(pap);

% Read and display the video frames
while hasFrame(videoObj_shortaxis_pap)
    videoFrame_shortaxis_pap = readFrame(videoObj_shortaxis_pap);
    imshow(videoFrame_shortaxis_pap);
    title_str = sprintf('The frame number is %d', n);
    title(title_str);    
    pause(p);
    n = n+1;        % Increment Frame Number 
end

% Frame to be read
frame_number = input('\nEnter the desired frame number: ');
fprintf('\nThe frame selected is %d', frame_number);

% Set the CurrentTime property to the desired frame
videoObj_shortaxis_pap.CurrentTime = (frame_number - 1) / videoObj_shortaxis_pap.FrameRate;

% Read the desired frame
frame = readFrame(videoObj_shortaxis_pap);

% Display the frame
imshow(frame);
title_str = sprintf('The frame number is %d', frame_number);
title(title_str);

% Select two points in the frame 
[x, y] = ginput(2);

% Define two points
point1 = [x(1), y(1)];
point2 = [x(2), y(2)];

% Calculate the Euclidean distance between the two points
shortaxis_pap = sqrt(sum(power(point1 - point2, 2)));

% Use scaling factor to get the distance value in cm
shortaxis_pap = shortaxis_pap/50;

% Print obtained value
fprintf('\nThe end systolic short axis papillary muscle value is %d', shortaxis_pap);

%% 

% Calculate the end systolic volume 
A1_endsystolic_mitralvalveshortaxis = pi*shortaxis_mitralvalve/4;
A2_endsystolic_papillarymusclelevel = pi*shortaxis_pap/4;
A3_endsystolic_apexshortaxis = pi*shortaxis_apex/4;

A1 = A1_endsystolic_mitralvalveshortaxis;
A2 = A2_endsystolic_papillarymusclelevel;
A3 = A3_endsystolic_apexshortaxis;

V_endsystolic = (A1 + A2)*h + (A3 * h)/2 + (pi * h^3)/6 ;


%% 

% Choose the frames and measure the end systolic volume 
fprintf('\nThe second measurement is done for the end diastolic volume. \nMeasurements must be made for the end diastolic volume only.');

% Pause to read dialogue
pause(2) 

% Frame number initiation
n = 1;              

% Create a VideoReader object
videoObj_apical4ch = VideoReader(apical);

% Read and display the video framesA3_enddiastolic_apexshortaxis
while hasFrame(videoObj_apical4ch)
    videoFrame_apical4ch = readFrame(videoObj_apical4ch);
    imshow(videoFrame_apical4ch);
    title_str = sprintf('The frame number is %d', n);
    title(title_str);    
    pause(p);
    n = n+1;        % Increment Frame Number 
end

% Frame to be read
frame_number = input('\nEnter the desired frame number: ');
fprintf('\nThe frame selected is %d', frame_number);

% Set the CurrentTime property to the desired frame
videoObj_apical4ch.CurrentTime = (frame_number - 1) / videoObj_apical4ch.FrameRate;

% Read the desired frame
frame = readFrame(videoObj_apical4ch);

% Display the frame
imshow(frame);
title_str = sprintf('The frame number is %d', frame_number);
title(title_str);

% Select two points in the frame 
[x, y] = ginput(2);

% Define two points
point1 = [x(1), y(1)];
point2 = [x(2), y(2)];

% Calculate the Euclidean distance between the two points
L = sqrt(sum(power(point1 - point2, 2)));
h = L/3;

% Use scaling factor to get the distance value in cm
h = h/10;

% Print obtained value
fprintf('\nLong axis end diastolic is %d', h);

%%
% Frame number initiation
n = 1;           

% Create a VideoReader object
videoObj_shortaxis_apex = VideoReader(apex);

% Read and display the video frames
while hasFrame(videoObj_shortaxis_apex)
    videoFrame_shortaxis_apex = readFrame(videoObj_shortaxis_apex);
    imshow(videoFrame_shortaxis_apex);
    title_str = sprintf('The frame number is %d', n);
    title(title_str);    
    pause(p);
    n = n+1;        % Increment Frame Number 
end

% Frame to be read
frame_number = input('\nEnter the desired frame number: ');
fprintf('\nThe frame selected is %d', frame_number);

% Set the CurrentTime property to the desired frame
videoObj_shortaxis_apex.CurrentTime = (frame_number - 1) / videoObj_shortaxis_apex.FrameRate;

% Read the desired frame
frame = readFrame(videoObj_shortaxis_apex);

% Display the frame
imshow(frame);
title_str = sprintf('The frame number is %d', frame_number);
title(title_str);

% Select two points in the frame 
[x, y] = ginput(2);

% Define two points
point1 = [x(1), y(1)];
point2 = [x(2), y(2)];

% Calculate the Euclidean distance between the two points
shortaxis_apex = sqrt(sum(power(point1 - point2, 2)));

% Use scaling factor to get the distance value in cm
shortaxis_apex = shortaxis_apex/48;

% Print obtained value
fprintf('\nThe end systolic short axis apex value is %d', shortaxis_apex);

%%
% Frame number initiation
n = 1;      

% Create a VideoReader object
videoObj_shortaxis_mitralvalve = VideoReader(mitralvalve);

% Read and display the video frames
while hasFrame(videoObj_shortaxis_mitralvalve)
    videoFrame_shortaxis_mitralvalve = readFrame(videoObj_shortaxis_mitralvalve);
    imshow(videoFrame_shortaxis_mitralvalve);
    title_str = sprintf('The frame number is %d', n);
    title(title_str);    
    pause(p);
    n = n+1;        % Increment Frame Number 
end

% Frame to be read
frame_number = input('\nEnter the desired frame number: ');
fprintf('\nThe frame selected is %d', frame_number);

% Set the CurrentTime property to the desired frame
videoObj_shortaxis_mitralvalve.CurrentTime = (frame_number - 1) / videoObj_shortaxis_mitralvalve.FrameRate;

% Read the desired frame
frame = readFrame(videoObj_shortaxis_mitralvalve);

% Display the frame
imshow(frame);
title_str = sprintf('The frame number is %d', frame_number);
title(title_str);

% Select two points in the frame 
[x, y] = ginput(2);

% Define two points
point1 = [x(1), y(1)];
point2 = [x(2), y(2)];

% Calculate the Euclidean distance between the two points
shortaxis_mitralvalve = sqrt(sum(power(point1 - point2, 2)));

% Use scaling factor to get the distance value in cm
shortaxis_mitralvalve = shortaxis_mitralvalve/48;

% Print obtained value
fprintf('\nThe end systolic short axis mitral valve value is %d', shortaxis_mitralvalve);

%%
% Frame number initiation
n = 1;        

% Create a VideoReader object
videoObj_shortaxis_pap = VideoReader(pap);

% Read and display the video frames
while hasFrame(videoObj_shortaxis_pap)
    videoFrame_shortaxis_pap = readFrame(videoObj_shortaxis_pap);
    imshow(videoFrame_shortaxis_pap);
    title_str = sprintf('The frame number is %d', n);
    title(title_str);    
    pause(p);
    n = n+1;        % Increment Frame Number 
end

% Frame to be read
frame_number = input('\nEnter the desired frame number: ');
fprintf('\nThe frame selected is %d', frame_number);

% Set the CurrentTime property to the desired frame
videoObj_shortaxis_pap.CurrentTime = (frame_number - 1) / videoObj_shortaxis_pap.FrameRate;

% Read the desired frame
frame = readFrame(videoObj_shortaxis_pap);

% Display the frame
imshow(frame);
title_str = sprintf('The frame number is %d', frame_number);
title(title_str);

% Select two points in the frame 
[x, y] = ginput(2);

% Define two points
point1 = [x(1), y(1)];
point2 = [x(2), y(2)];

% Calculate the Euclidean distance between the two points
shortaxis_pap = sqrt(sum(power(point1 - point2, 2)));

% Use scaling factor to get the distance value in cm
shortaxis_pap = shortaxis_pap/50;

% Print obtained value
fprintf('\nThe end systolic short axis papillary muscle value is %d', shortaxis_pap);

%% 
% Calculate the end diastolic volume 

A1_enddiastolic_mitralvalveshortaxis = pi*shortaxis_mitralvalve/4;
A2_enddiastolic_papillarymusclelevel = pi*shortaxis_pap/4;
A3_enddiastolic_apexshortaxis = pi*shortaxis_pap/4;

A1 = A1_enddiastolic_mitralvalveshortaxis;
A2 = A2_enddiastolic_papillarymusclelevel;
A3 = A3_enddiastolic_apexshortaxis;

V_enddiastolic = (A1 + A2)*h + (A3 * h)/2 + (pi * h^3)/6 ;

%% 

% Calculate the required cardiac parameters 

HR = 60;

Stroke_volume = V_enddiastolic - V_endsystolic;
Ejection_fraction = Stroke_volume/ V_enddiastolic;
Cardiac_output = Stroke_volume * HR;

fprintf('\nThe stroke volume is %d cc. \n', Stroke_volume);
fprintf('The ejection fraction is %d. \n', Ejection_fraction);
fprintf('The cardiac output is %d cc. \n', Cardiac_output);

SV = Stroke_volume;
ESV = V_endsystolic;
EDV = V_enddiastolic;
EF = Ejection_fraction;
CO = Cardiac_output;


end







