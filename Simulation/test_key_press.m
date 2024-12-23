% Create a figure
hFig = figure;

% Set the KeyPressFcn callback
set(hFig, 'KeyPressFcn', @keyPressCallback);


% Callback function to handle key presses
function keyPressCallback(~, event)
    % Display the key that was pressed
    fprintf('Key pressed: %s\n', event.Key);
    
    % You can add specific actions for certain keys
    switch event.Key
        case 'a'
            disp('You pressed "a"!');
        case 'escape'
            disp('Exiting program.');
            close(gcf); % Close the figure
        otherwise
            disp('Other key pressed.');
    end
end
