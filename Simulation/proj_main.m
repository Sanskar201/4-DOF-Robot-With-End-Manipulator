% MAE 263A Project
% Simulation

clc;
% clf;
clear all;

%% Set path parameters
% offset = [2048, 2151, 1952, 2239];
% joint_offset = 4096/2/pi * (offset - 2048);

% Trajectory Cartesian Space
N = 101;
t = linspace(0,2*pi,N);
x = 0.025*cos(t) + 0.15;
y = 0.025*sin(t);
z = ones(1,N)*0;
% R = [-1 0 0;0 1 0;0 0 -1];
% orient = -pi/2 * ones(1,N);
orient = -pi/2 - 0.3*cos(t);

%% Keyboard input parameters

global x_mov y_mov z_mov orient_ref c escape_pressed q step step_val


% Main Parameter
l1 = 0.06;  % m
l2 = 0.1086;  % m
l3 = 0.1086;  % m
l4 = 0.1086;  % m
c = [l1 l2 l3 l4];

% Coordinate inputs
N = 3;
escape_pressed = 0;
x_mov = [0.1, zeros(1,N-1)];
y_mov = zeros(1,N);
z_mov = zeros(1,N);
orient_ref = -pi/2;
step = 1;
step_val = 0.01;




[X, Y, Z] = sphere;
r = 0.05;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

figure(1)
hold on
% view(45, 45)
% surf(X2, Y2, Z2, "FaceAlpha",0.1);
% shading interp
stop_flag = 0;

while stop_flag == 0
    % Create a figure
    k = waitforbuttonpress;
    value = double(get(gcf,'CurrentCharacter'));
    if value == 113
        z_mov(step + 1) = z_mov(step);
        y_mov(step + 1) = y_mov(step);
        x_mov(step + 1) = x_mov(step) + step_val;
        disp('moving along +x' + string(x_mov(step + 1)))
        %step = step + 1;
    elseif value == 119
        z_mov(step + 1) = z_mov(step);
        x_mov(step + 1) = x_mov(step);
        y_mov(step + 1) = y_mov(step) + step_val;
        disp('moving along +y' + string(y_mov(step + 1)))
        %step = step + 1;
    elseif value == 101
        x_mov(step + 1) = x_mov(step);
        y_mov(step + 1) = y_mov(step);
        z_mov(step + 1) = z_mov(step) + step_val;
        disp('moving along +z' + string(z_mov(step + 1)))
        %step = step + 1;
    elseif value == 97
        z_mov(step + 1) = z_mov(step);
        y_mov(step + 1) = y_mov(step);
        x_mov(step + 1) = x_mov(step) - step_val;
        disp('moving along -x' + string(x_mov(step + 1)))
        %step = step + 1;
    elseif value == 115
        z_mov(step + 1) = z_mov(step);
        x_mov(step + 1) = x_mov(step);
        y_mov(step + 1) = y_mov(step) - step_val;
        disp('moving along -y' + string(y_mov(step + 1)))
        %step = step + 1;
    elseif value == 100
        x_mov(step + 1) = x_mov(step);
        y_mov(step + 1) = y_mov(step);
        z_mov(step + 1) = z_mov(step) - step_val;
        disp('moving along -z' + string(z_mov(step + 1)))
        %step = step + 1;
    elseif value == 27
        disp('Exiting program.');
        close(gcf); % Close the figure
        x_mov = x_mov(1:step);
        y_mov = y_mov(1:step);
        z_mov = z_mov(1:step);
        stop_flag = 1;
        break;
    else
    end
    % check workspace boudaries 
    % x_mov
    % y_mov
    % z_mov
    q = proj_IK([x_mov(step), y_mov(step), z_mov(step)], orient_ref, c);
    if ismember(-100, q)
        disp(" increment goes past workspace boundaries")
        break
    else
        plot_frame(c,q)
        step = step + 1;
        disp('Press escape to stop the program')
    end

end

hold off
close(gcf); % Close the figure



for i = 1:size(x_mov, 2)
    p = [x_mov(i) y_mov(i) z_mov(i)]';
    % T0e = [R p;0 0 0 1];
    q(i, :) = proj_IK(p, orient_ref, c);
end

theta1 = q(:, 1)';
theta2 = q(:, 2)';
theta3 = q(:, 3)';
theta4 = q(:, 4)';

t1 = unwrap(theta1);
t2 = unwrap(theta2);
t3 = unwrap(theta3);
t4 = unwrap(theta4);

joint = [t1;t2;t3;t4];
path = [x_mov;y_mov;z_mov];

movie = 0; % create movie if 1
speed = 1; % 1 to N

figure(2)
for i = 1:1
    proj_animation(c,joint,path,movie,speed)
end

%% Test

q = [0.01, 0.01, 0.01, 0.01];
plot_frame(c,q)

%% Helper functions

% Callback function to handle key presses
function keyPressCallback(~, event)
    % Display the key that was pressed
    % fprintf('Key pressed: %s\n', event.Key);
    global x_mov y_mov z_mov orient_ref c escape_pressed q step

    
    % You can add specific actions for certain keys
    switch event.Key
        case 'x'
            x_mov(step + 1) = x_mov(step) + 0.01;
            disp('moving along +x' + string(x_mov(step + 1)))
        case 'y'
            y_mov(step + 1) = y_mov(step) + 0.01;
            disp('moving along +y' + string(y_mov(step + 1)))
        case 'z'
            z_mov(step + 1) = z_mov(step) + 0.01;
            disp('moving along +z' + string(z_mov(step + 1)))
        case 's'
            x_mov(step + 1) = x_mov(step) - 0.01;
            disp('moving along -x' + string(x_mov(step + 1)))
        case 'j'
            y_mov(step + 1) = y_mov(step) - 0.01;
            disp('moving along -y' + string(y_mov(step + 1)))
        case 'k'
            z_mov(step + 1) = z_mov(step) - 0.01;
            disp('moving along -z' + string(z_mov(step + 1)))
        case 'escape'
            disp('Exiting program.');
            close(gcf); % Close the figure
            escape_pressed = 1;
    end

    step = step + 1;
    % disp(x_mov)
    % x_mov = clip(x_mov, -pi, pi);
    % y_mov = clip(y_mov, -pi, pi);
    % z_mov = clip(z_mov, -pi, pi);
    % q = proj_IK([x, y, z], orient_ref, c);
    % plot_frame(c, q)
    % disp('t1: ' +  string(q(1)))
end

function plot_frame(c, q)
    [fx,fy,fz,T] = proj_FK(c,q);
%     fz = fz + 0.1;
    [n_joint, ~] = size(q);

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
    axis([-0.35 0.35 -0.35 0.35 0.0 0.4]);
    pbaspect([1 1 1]);
    grid on;
%     view(0,0);
    % view(40,30);
    hold off;
    drawnow;
end