% Clear workspace and command window
clear all;
clc;

% Setup serial connection
global arduino; % Declare as global to be used in the callback
arduino = serialport('COM4', 9600);
configureTerminator(arduino, "CR/LF");
flush(arduino);

% Give Arduino time to initialize
pause(2);

% Create figure window for keyboard input
fig = figure('KeyPressFcn', @keyPressed);
title('Servo Control Window');
xlabel('Press A or S to control servo. Press Q to quit.');

% Global variable to control program execution
global isRunning;
isRunning = true;

% Callback function for key presses
function keyPressed(~, event)
    global arduino;  % Access the global Arduino object
    global isRunning;
    
    try
        switch event.Key
            case 'a'
                write(arduino, 'a', 'char');
                disp('Moving servo forward');
            case 's'
                write(arduino, 's', 'char');
                disp('Moving servo backward');
            case 'q'
                isRunning = false;
                disp('Closing program...');
        end
    catch ME
        disp('Error in keyPressed function:');
        disp(ME.message);
    end
end

% Main program loop
while isRunning
    pause(0.1); % Prevent excessive CPU usage
end

% Cleanup
clear arduino;
close all;
