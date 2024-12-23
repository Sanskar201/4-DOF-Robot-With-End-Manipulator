function proj_animation(c,joint,path,movie,speed)
% joint (4xN): [t1 t2 t3 t4]'
% path (3xN): [x y z]'
l1 = c(1);
l2 = c(2);
l3 = c(3);
l4 = c(4);
px = path(1,:);
py = path(2,:);
pz = path(3,:);
[n_joint, ~] = size(joint);

% Create Movie
if movie == 1
    v = VideoWriter('ani.avi');
    open(v);
end

for i = 1:speed:length(joint)
    
    [fx,fy,fz,T] = proj_FK(c,joint(:,i));
%     fz = fz + 0.1;
    
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
    % Trajectory
    plot3(px,py,pz,'b');
    % Ground
    X = [1 -1;1 -1]*0.2;
    Y = [1 1;-1 -1]*0.2;
    Z = [1 1;1 1]*0;
    surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none'); hold on;
    % Label
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    axis([-0.2 0.2 -0.2 0.2 0.0 0.4]);
    pbaspect([1 1 1]);
    grid on;
%     view(0,0);
    % view(40,30);
    hold off;
    drawnow;
    
    if movie == 1
    frame = getframe(gcf);
    writeVideo(v,frame);
    end

end

if movie == 1
    close(v)
end

end