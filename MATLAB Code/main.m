% MAE C163A/C263A Project
% Team 12

% Initialize Motor
clear arduino;
clear all;
initialize();

% Main

% Initialized arduino related variables
global arduino; % Declare as global to be used in the callback
arduino = serialport('COM4', 9600);
configureTerminator(arduino, "CR/LF");
flush(arduino);

% Keyboard Mapping for manual control
% A/D: rotate robot CCW/CW (+/- theta1)
% W/S: reach forward/backward (+/- r_proj)
% J/K: lower/raise end effector (-/+ z_ref)
% N/M: move prismatic left/right (+/- y_robot_base)
% 1/2: close/open gripper

% Set up position variables
global theta_ref r_proj_ref z_ref orient_ref c q escape_pressed d_ref

% Keypress variables
global pressA pressD pressW pressS pressJ pressK press1 press2 pressN pressM

% Position and offset set up
escape_pressed = 0;
theta_ref = 0;
r_proj_ref = 0.15;
z_ref = 0;
d_ref = 0;
orient_ref = -pi/2;
c = [0.06 0.1086 0.1086 0.1086 0.2005];
[x,y,z] = pol2cart(theta_ref,r_proj_ref,z_ref);
q = proj_IK([x,y,z], orient_ref, c);
q = [q, 0];  % initialize prismatic to 0 position
offset = [2048, 2151, 1952, 2239, 2389];

for i = 1:5
    % Write speed constraint for the motors last input: [0 is no speed 
    % control, 1 is lowest speed], [1 1000] is slowest to fastest mapping 
    % for values other than [0, 1]
    write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), 112, 600);
end

% Create a figure
hFig = figure;

% Set the KeyPressFcn callback
set(hFig, 'KeyPressFcn', @keyPressCallback);
set(hFig, 'KeyReleaseFcn', @keyReleaseCallback);

q_int = clip(round(q*4096/2/pi + offset), 0, 4095);

% Go to initial position (not zero)
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), MX28_GOAL_POSITION, typecast(int32(q_int(1)), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(2), MX28_GOAL_POSITION, typecast(int32(q_int(2)), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(3), MX28_GOAL_POSITION, typecast(int32(q_int(3)), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(4), MX28_GOAL_POSITION, typecast(int32(q_int(4)), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(5), MX28_GOAL_POSITION, typecast(int32(q_int(5)), 'uint32'));

disp(bitget(read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), 10),2));
%write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), 10, bitset(read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), 10),2,1));
disp(bitset(read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), 10),2,1));

disp(read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), 10));

% Movement Tracking Variable Initialization
ppos = [0 0 0 0 0];
for i = 1:5
    ppos(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), 132);
end

% Tracking
count = 0;
mode = input('Select Mode: 1 = Predefined Task | 2 = Manual Control');

if mode == 1
    disp("working")
    % pos = [x;y;z]
    p1 = [0.15,0,0.055]; % Raise Up
    p2 = [0.12721,0.22786,0.055]; % Move to Butter
    p3 = [0.12721,0.22786,-0.025]; % Decscend
    p4 = p2; % Ascsend
    p5 = p1; % Move to Drop Off
    p6 = [-0.006,-0.18,0.055]; % Move to Drop Off
    p7 = [-0.006,-0.18,0.01]; % Descend
    p8 = p1; % Move Back to Start
    p9 = [0.15,0,0];
    steps = [p1;p2;p3;p4;p5;p6;p7;p8;p9];
    
    for step=1:9
        % Update goal to next position
        q = proj_IK_with_slider(steps(step,:), orient_ref, c);
        q_int = clip(round(q*4096/2/pi + offset), 0, 4095);
        
        % Write new position goal
        write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), MX28_GOAL_POSITION, typecast(int32(q_int(1)), 'uint32'));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(2), MX28_GOAL_POSITION, typecast(int32(q_int(2)), 'uint32'));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(3), MX28_GOAL_POSITION, typecast(int32(q_int(3)), 'uint32'));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(4), MX28_GOAL_POSITION, typecast(int32(q_int(4)), 'uint32'));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(5), MX28_GOAL_POSITION, typecast(int32(q_int(5)), 'uint32'));
        
        % Wait for goal reach
        moving = true;
        while moving
            moving = false;
            for i = 1:5
                cpos = read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), 132);
            
                % Checks previous position against current position
                if abs(ppos(i) - cpos) > 2
                    moving = true;
                    ppos(i) = cpos;
                end
            end
            
            % plot_frame_with_slider(c, q)
            plot_frame_with_slider(c, (ppos - offset)/4096*2*pi)

            pause(0.01)
        end
        
        % Close gripper once at p3
        if step == 3
            write(arduino, "a", 'char');
            pause(0.8);
        end
        % Open gripper once at p6
        if step == 7
            write(arduino, "s", 'char');
            write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), 112, 900);
            pause(0.8);
        end
    end

    while not(escape_pressed)
        pause(0.1);
    end
else
    % Mode 2
    while true
        if escape_pressed
            break;
        end
        
        moving = false;
        for i = 1:5
            cpos = read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), 132);
    
            % Allows for movement if close to goal
            if abs(ppos(i) - cpos) > 10
                moving = true;
                ppos(i) = cpos;
            end
        end
        
        % Tracking
        %disp(strcat(num2str(count),strcat(" | ",num2str(ppos(1)))))
        %count = count+1;
    
        % Only update reference when the robot finish moving
        if not(moving)
            % Tracking
            %disp("move")
            %count = 0;
    
            if pressA
                theta_ref = theta_ref + 2*pi/180;
                % disp('pressA')
            end
            if pressD
                theta_ref = theta_ref - 2*pi/180;
                % disp('pressD')
            end
            if pressW
                r_proj_ref = r_proj_ref + 0.005;
                % disp('pressW')
            end
            if pressS
                r_proj_ref = r_proj_ref - 0.005;
                % disp('pressS')
            end
            if pressJ
                z_ref = z_ref - 0.005;
                % disp('pressJ')
            end
            if pressK
                z_ref = z_ref + 0.005;
                % disp('pressK')
            end
            if pressN
                d_ref = d_ref + 0.005;
            end
            if pressM
                d_ref = d_ref - 0.005;
            end
            if press1
                write(arduino, 'a', 'char');
                disp('Closing Gripper');
            end
            if press2
                write(arduino, 's', 'char');
                disp('Opening Gripper');
            end
    
            theta_ref = clip(theta_ref, -pi, pi);
            r_proj_ref = clip(r_proj_ref, 0.05, 0.2);
            z_ref = clip(z_ref, -0.05, 0.13);
            d_ref = clip(d_ref, 0, 0.74*c(5));
            [x,y,z] = pol2cart(theta_ref, r_proj_ref, z_ref);
            disp("[x, y, z]: ("+string(x)+" "+string(y+d_ref)+" "+string(z)+")")
            q = proj_IK([x, y, z], orient_ref, c);
            q_pris = map_pris2angle(d_ref, c);

            q = [q, q_pris];
            q_int = clip(round(q*4096/2/pi + offset), 0, 4095);

            plot_frame_with_slider(c, q)
            
            
            % Move motors to calculated angles
            write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(1), MX28_GOAL_POSITION, typecast(int32(q_int(1)), 'uint32'));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(2), MX28_GOAL_POSITION, typecast(int32(q_int(2)), 'uint32'));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(3), MX28_GOAL_POSITION, typecast(int32(q_int(3)), 'uint32'));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(4), MX28_GOAL_POSITION, typecast(int32(q_int(4)), 'uint32'));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(5), MX28_GOAL_POSITION, typecast(int32(q_int(5)), 'uint32'));
            
            for i = 1:5
                goalPos(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), MX28_GOAL_POSITION);
            end
        end
        
        pause(0.01)
    end
end

% Terminate
terminate();
clear arduino;

% Callback function to handle key presses
function keyReleaseCallback(~, event)
    global pressA pressD pressW pressS pressJ pressK press1 press2 pressN pressM
    
    switch event.Key
        case 'a'
            % Increase heading reference
            pressA = false;
        case 'd'
            % Decrease heading reference
            pressD = false;
        case 'w'
            % Increase reach distance
            pressW = false;
        case 's'
            % Decrease reach distance
            pressS = false;
        case 'j'
            % Decrease elevation
            pressJ = false;
        case 'k'
            % Increase elevation
            pressK = false;
        case 'n'
            % Moving toward close extreme prismatic config
            pressN = false;
        case 'm'
            % Moving toward far extreme prismatic config
            pressM = false;
        case '1'
            % Close Gripper
            press1 = false;
        case '2'
            % Open Gripper
            press2 = false;
    end
end

function keyPressCallback(~, event)
    global pressA pressD pressW pressS pressJ pressK press1 press2 escape_pressed pressN pressM
    
    switch event.Key
        case 'a'
            % Increase heading reference
            pressA = true;
        case 'd'
            % Decrease heading reference
            pressD = true;
        case 'w'
            % Increase reach distance
            pressW = true;
        case 's'
            % Decrease reach distance
            pressS = true;
        case 'j'
            % Decrease elevation
            pressJ = true;
        case 'k'
            % Increase elevation
            pressK = true;
        case 'n'
            % Moving toward close extreme prismatic config
            pressN = true;
        case 'm'
            % Moving toward far extremem prismatic config
            pressM = true;
        case '1'
            % Close Gripper
            press1 = true;
        case '2'
            % Open Gripper
            press2 = true;
        case 'escape'
            disp('Exiting program.');
            close(gcf); % Close the figure
            escape_pressed = 1;
    end
end

function plot_frame(c, q)
    [fx,fy,fz,T] = proj_FK(c,q);
%     fz = fz + 0.1;
    [n_joint, ~] = size(q);
    subplot(121)
    % Base
    plot3([0 fx(1)],[0 fy(1)],[0 fz(1)],'k','linewidth',8);
    hold on;
    % Manipulator
    plot3(fx(1:end-1),fy(1:end-1),fz(1:end-1),'k','linewidth',4);
    % Tool
    plot3(fx(end-1:end),fy(end-1:end),fz(end-1:end),'m','linewidth',3);
    % Frames
    for j = 1:(n_joint+2)
        Rj = T{j}(1:3,1:3);
        mag = 0.025;
        plot3(fx(j)+[0 Rj(1,1)]*mag,fy(j)+[0 Rj(2,1)]*mag,fz(j)+[0 Rj(3,1)]*mag,'r','linewidth',2); % x
        plot3(fx(j)+[0 Rj(1,2)]*mag,fy(j)+[0 Rj(2,2)]*mag,fz(j)+[0 Rj(3,2)]*mag,'g','linewidth',2); % y
        plot3(fx(j)+[0 Rj(1,3)]*mag,fy(j)+[0 Rj(2,3)]*mag,fz(j)+[0 Rj(3,3)]*mag,'b','linewidth',2); % z
    end
    % % Trajectory
    % plot3(px,py,pz,'b');
    % Ground
    X = [1 -1;1 -1]*0.2;
    Y = [1 1;-1 -1]*0.2;
    Z = [1 1;1 1]*0;
    surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none'); hold on;
    % Label
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Top View')
    axis([-0.35 0.35 -0.35 0.35 0.0 0.4]);
    pbaspect([1 1 1]);
    grid on;
    view(-90,90);
    % view(40,30);
    hold off;

    subplot(122)
    plot_frame_side_view(fx, fy, fz, T, n_joint)
    drawnow;
end

function plot_frame_side_view(fx,fy,fz,T, n_joint)
%     [fx,fy,fz,T] = proj_FK(c,q);
% %     fz = fz + 0.1;
%     [n_joint, ~] = size(q);

    % Base
    plot3([0 fx(1)],[0 fy(1)],[0 fz(1)],'k','linewidth',8);
    hold on;
    % Manipulator
    plot3(fx(1:end-1),fy(1:end-1),fz(1:end-1),'k','linewidth',4);
    % Tool
    plot3(fx(end-1:end),fy(end-1:end),fz(end-1:end),'m','linewidth',3);
    % Frames
    for j = 1:(n_joint+2)
        Rj = T{j}(1:3,1:3);
        mag = 0.025;
        plot3(fx(j)+[0 Rj(1,1)]*mag,fy(j)+[0 Rj(2,1)]*mag,fz(j)+[0 Rj(3,1)]*mag,'r','linewidth',2); % x
        plot3(fx(j)+[0 Rj(1,2)]*mag,fy(j)+[0 Rj(2,2)]*mag,fz(j)+[0 Rj(3,2)]*mag,'g','linewidth',2); % y
        plot3(fx(j)+[0 Rj(1,3)]*mag,fy(j)+[0 Rj(2,3)]*mag,fz(j)+[0 Rj(3,3)]*mag,'b','linewidth',2); % z
    end
    % % Trajectory
    % plot3(px,py,pz,'b');
    % Ground
    X = [1 -1;1 -1]*0.2;
    Y = [1 1;-1 -1]*0.2;
    Z = [1 1;1 1]*0;
    surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none'); hold on;
    % Label
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Side View')
    axis([-0.35 0.35 -0.35 0.35 0.0 0.4]);
    pbaspect([1 1 1]);
    grid on;
    view(-90,0);
    % view(40,30);
    hold off;
    drawnow;
end

function plot_frame_with_slider(c, q)
    pris_d = map_angle2pris(q(5), c);
    [fx,fy,fz,T] = proj_FK(c,q(1:4));
    fy = fy + pris_d;
%     fz = fz + 0.1;
    n_joint = 4;
    subplot(121)
    % Base
    plot3([0 fx(1)],[0 fy(1)],[0 fz(1)],'k','linewidth',8);
    hold on;
    % Manipulator
    plot3(fx(1:end-1),fy(1:end-1),fz(1:end-1),'k','linewidth',4);
    % Tool
    plot3(fx(end-1:end),fy(end-1:end),fz(end-1:end),'m','linewidth',3);
    % Frames
    for j = 1:(n_joint+2)
        Rj = T{j}(1:3,1:3);
        mag = 0.025;
        plot3(fx(j)+[0 Rj(1,1)]*mag,fy(j)+[0 Rj(2,1)]*mag,fz(j)+[0 Rj(3,1)]*mag,'r','linewidth',2); % x
        plot3(fx(j)+[0 Rj(1,2)]*mag,fy(j)+[0 Rj(2,2)]*mag,fz(j)+[0 Rj(3,2)]*mag,'g','linewidth',2); % y
        plot3(fx(j)+[0 Rj(1,3)]*mag,fy(j)+[0 Rj(2,3)]*mag,fz(j)+[0 Rj(3,3)]*mag,'b','linewidth',2); % z
    end
    % % Trajectory
    % plot3(px,py,pz,'b');
    % Ground
    X = [1 -1;1 -1]*0.2;
    Y = [1 1;-1 -1]*0.2;
    Z = [1 1;1 1]*0;
    surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none'); hold on;
    % Label
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Top View')
    axis([-0.35 0.35 -0.35 0.35 0.0 0.4]);
    pbaspect([1 1 1]);
    grid on;
    view(-90,90);
    % view(40,30);
    hold off;

    subplot(122)
    plot_frame_side_view(fx, fy, fz, T, n_joint)
    drawnow;
end