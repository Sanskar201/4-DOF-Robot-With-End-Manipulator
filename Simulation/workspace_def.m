
clear all;
clc;


%%
% Plotting Workspace


step = 271;
x_step = 100;
step_a=0.01;        %step angle

% Main Parameter
l1 = 0.06;  % m
l2 = 0.1086;  % m
l3 = 0.1086;  % m
l4 = 0.1086;  % m
c = [l1 l2 l3 l4];
orient_ref = deg2rad(linspace(-135, 135, step));

% Angle limits (subject to change with CAD geometry)
t1lim = (pi/180).*[-180, 180];
%t2lim = (pi/180).*[-15, 135];
t2lim = (pi/180).*[-135, 15];
t3lim = (pi/180).*[-90, 135];
t4lim = (pi/180).*[-135, 135];

tmin = [t1lim(1), t2lim(1), t3lim(1), t4lim(1)];
tmax = [t1lim(2), t2lim(2), t3lim(2), t4lim(2)];


% Equation for dome
dome = @(y) sqrt((l2+l3+l4).^2 - (y-l1).^2);

y = linspace(0, (l1+l2+l3+l4), step);
x_pts = zeros(1, step*step*x_step);
y_pts = zeros(1, step*step*x_step);

%%
% Plotting boundaries of workspace (messy)

itt = 0;
y_bound = linspace(0, l1+l2+l3+l4, step);


for tt = 1: step
    for ii = 1:step-1
        y_test = y(ii);
        x_barr = dome(y_test);
        x_test = linspace(0, x_barr, x_step);
        last_flag = 0;

        for jj = 1:x_step
            x_test(jj);
            q = proj_IK([x_test(jj), 0, y_test], orient_ref(tt), c);
            flag = 0;
            for kk = 1:4
                flag = flag + (tmin(kk) > q(kk)) + (tmax(kk) < q(kk));                
            end 
            flag = flag > 0;

            if flag ~= last_flag
                %disp("flag: " + string(flag))
                %disp("last_flag" + string(last_flag))
                x_pts( (x_step*(step-1)*tt) + (x_step*ii) + jj ) = x_test(jj);
                y_pts( (x_step*(step-1)*tt) + (x_step*ii) + jj ) = y_test;
                last_flag = flag;
            else
                %disp("equal")
            end
            last_flag = flag;
        end
    end
    tt;
    if mod(round(rad2deg(orient_ref(tt))), 45) == 0 && rad2deg(orient_ref(tt)) >= round(rad2deg(orient_ref(tt))) 
        %disp('save')
        file_name = 'Workspace boundaries of ' + string(rad2deg(orient_ref(tt)));
        clf;
        fig = figure;
        hold on
        plot(real(dome(y_bound)), y_bound, 'LineWidth', 3)
        scatter(x_pts((x_step*(step-1)*tt):(x_step*(step-1)*(tt+1))), ...
            y_pts((x_step*(step-1)*tt):(x_step*(step-1)*(tt+1))))
        title('Workspace boundaries of orientation at ' + string(rad2deg(orient_ref(tt))))
        hold off
        savefig(fig,file_name + '.fig')
        close(fig)
    end
    disp(round(rad2deg(orient_ref(tt))))
end


% clf;
% figure(1)
% hold on
% plot(real(dome(y_bound)), y_bound, 'LineWidth', 3)
% scatter(x_pts, y_pts)
% hold off

%%
% Plotting space of workspace

itt = 0;
y_bound = linspace(0, l1+l2+l3+l4, step);


for tt = 1: step
    for ii = 1:step-1
        y_test = y(ii);
        x_barr = dome(y_test);
        x_test = linspace(0, x_barr, x_step);

        for jj = 1:x_step
            x_test(jj);
            q = proj_IK([x_test(jj), 0, y_test], orient_ref(tt), c);
            flag = 0;
            for kk = 1:4
                flag = flag + (tmin(kk) > q(kk)) + (tmax(kk) < q(kk));                
            end 

            if flag == 0
                x_pts( (x_step*(step-1)*tt) + (x_step*ii) + jj ) = x_test(jj);
                y_pts( (x_step*(step-1)*tt) + (x_step*ii) + jj ) = y_test;                
            else
            end
        end
    end
    tt;
    if mod(round(rad2deg(orient_ref(tt))), 45) == 0 && rad2deg(orient_ref(tt)) >= round(rad2deg(orient_ref(tt))) 
        tt
        file_name = 'Workspace of ' + string(rad2deg(orient_ref(tt)));
        clf;
        fig = figure;
        hold on
        plot(real(dome(y_bound)), y_bound, 'LineWidth', 3)
        scatter(x_pts((x_step*(step-1)*tt):(x_step*(step-1)*(tt+1))), ...
            y_pts((x_step*(step-1)*tt):(x_step*(step-1)*(tt+1))))
        title('Workspace of orientation at ' + string(rad2deg(orient_ref(tt))))
        hold off
        savefig(fig,file_name + '.fig')
        close(fig)
    end
    disp(round(rad2deg(orient_ref(tt))))
end

% Plotting all reachable areas with orientations
clf;
figure(2)
hold on
plot(real(dome(y_bound)), y_bound, 'LineWidth', 3)
%scatter(x_pts, y_pts)
scatter(x_pts((x_step*(step-1)):(x_step*(step-1)*45)), ...
                y_pts((x_step*(step-1)):(x_step*(step-1)*45)))
for kk = 1:5
    scatter(x_pts((x_step*(step-1)*(kk*45)):(x_step*(step-1)*((kk+1)*45))), ...
                y_pts((x_step*(step-1)*(kk*45)):(x_step*(step-1)*((kk+1)*45))))
end

hold off



%%
% Testing specific orientations


y_bound = linspace(0, l1+l2+l3+l4, step);
figure(2)
plot(real(dome(y_bound)), y_bound)
disp('this is number: ' + string(3))

% Testing certain orientation
test_x_pts = zeros(1, step*x_step);
test_y_pts = zeros(1, step*x_step);
test_ref = deg2rad(-70);

for ii = 1:step-1
    y_test = y(ii);
    x_barr = dome(y_test);
    x_test = linspace(0, x_barr, x_step);

    for jj = 1:x_step
        x_test(jj);
        q = proj_IK([x_test(jj), 0, y_test], test_ref, c);
        test = zeros(1,3);
        flag = 0;
        for kk = 1:4
            flag = flag + (tmin(kk) > q(kk)) + (tmax(kk) < q(kk));
        end 
        if flag == 0
            %disp('unable')
            test_x_pts((x_step*ii) + jj ) = x_test(jj);
            test_y_pts((x_step*ii) + jj ) = y_test;
        else
        end
        %last_flag = flag;
    end
end

clf;
figure(1)
hold on
plot(real(dome(y_bound)), y_bound, 'LineWidth', 3)
scatter(test_x_pts, test_y_pts)
hold off

%%
% Testing orienta