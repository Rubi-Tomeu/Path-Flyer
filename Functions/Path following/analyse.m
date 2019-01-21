%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path Following Evaluation %
% Tomeu Rubí                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all

% path_type=1;
% z_var=1;
% g_end=1;

%Path
[x,y,z]=CreatePath(path_type,z_var,Amp_path);
long = round(g_end*2*pi*1000);
if path_type == 4
    long = long + 1000;
end


%number of samples and total time
[n,~]=size(ErrX.data);
Tfinal = ErrX.time(n);
deltaT = horzcat(0,diff(ErrX.time)');

%First sample
n1=1;
Tinici = ErrX.time(n1);

%3D TRajectory
if strcmp(Traj_plot,'on')
    figure;
    hold on
    plot3(x(1:long),y(1:long),z(1:long),'--','LineWidth',1.5,'color', 'black');
    plot3(yout(:,10),yout(:,11),yout(:,12),'LineWidth',1,'color','red');
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    legend('Path Command','Trajectory');
    box on
    grid on
    if z_var == 0
        axis equal
    end
    view(60,30);
    pbaspect([1 1 0.5]);
    set(gcf,'color','white')
    set(gca,'FontName','euclid')
    % export_fig 3DNLGL.png -m5
    hold off
end

%Plot Errors
if strcmp(ErrX_plot,'on')
    figure;
    plot(ErrX.time,ErrX.data);
    title('Error on X (m)');
    xlabel('Time (s)');
end
if strcmp(ErrY_plot,'on')
    figure;
    plot(ErrY.time,ErrY.data);
    title('Error on Y (m)');
    xlabel('Time (s)');
end
if strcmp(ErrZ_plot,'on')
    figure;
    plot(ErrZ.time,ErrZ.data);
    title('Error on Z (m)');
    xlabel('Time (s)');
end

%Calculate norms
L1_norm = zeros(1,n);
L2_norm_2D = L1_norm;
L2_norm_3D = L1_norm;
L_Infinity_norm = L1_norm;
Control_effort = L1_norm;
yaw_ref = yout(:,23)';
yaw=yout(:,6)';
for i = 1:1:n
    %L1 norm
    L1_norm(i) = abs(ErrX.data(i)) + abs(ErrY.data(i)) + abs(ErrZ.data(i));
    %L2 norm(2D) - Euclidean distance
    L2_norm_2D(i) = sqrt((ErrX.data(i)).^2+(ErrY.data(i)).^2);
    %L2 norm(3D) - Euclidean distance
    L2_norm_3D(i) = sqrt((ErrX.data(i)).^2+(ErrY.data(i)).^2+(ErrZ.data(i)).^2);
    %L infinity norm
    L_Infinity_norm(i) = max([abs(ErrX.data(i)) abs(ErrY.data(i)) abs(ErrZ.data(i))]);
    %Control effort
    Control_effort(i) = (yout(i,17) + yout(i,18) + yout(i,19) + yout(i,20))/4/2;
    % Correct extra "turns"
    nv=floor((yaw(i) - yaw_ref(i))/(2*pi));
    if nv < 0
        nv = nv + 1;
    end
    yaw_ref(i) = yaw_ref(i) + nv*2*pi;

    % Nearest angle
    if abs(yaw(i) - yaw_ref(i)) > pi
        if yaw_ref(i) > yaw(i)
            yaw_ref(i) = yaw_ref(i) - 2*pi;
        else
            yaw_ref(i) = yaw_ref(i) + 2*pi;
        end
    end
end

%Plot L2 Norm
if strcmp(L2_plot,'on')
    figure;
    plot(ErrX.time,L2_norm_3D);
    title('L2 Norm - 3D');
    xlabel('Time (s)');
end

%Plot Yaw Error
if strcmp(YawErr_plot,'on')
    figure;
    plot(tout,abs(yaw_ref-yaw)*180/pi);
    title('Absolute Yaw Error (deg)');
    xlabel('Time (s)');
end

% Velocity module
if strcmp(Vel_plot,'on')
    figure;
    plot(tout,sqrt(yout(:,7).^2+yout(:,8).^2+yout(:,9).^2));
    title('Velocity (m/s)');
    xlabel('Time (s)');
end

%Plot other norms
% figure;
% plot(ErrX.time,L1_norm);
% title('Norma L1');
% xlabel('Temps (s)');
% 
% figure;
% plot(ErrX.time,L2_norm_2D);
% title('Norma L2 - 2D');
% xlabel('Temps (s)');

% Control Effort
if strcmp(CEff_plot,'on')
    figure;
    plot(tout,Control_effort);
    title('Control effort (%)');
    xlabel('Time (s)');
end

%Plot algorithm parameter
if strcmp(AlgParam_plot,'on')
    figure;
    plot(param_alg.time,param_alg.data);
    title('Algorithm parameter');
    xlabel('Time (s)');
end

%Calculate mean of norms
meanL1 = sum(L1_norm.*deltaT)/(Tfinal-Tinici);
meanL2_2D = sum(L2_norm_2D.*deltaT)/(Tfinal-Tinici);
meanL2_3D = sum(L2_norm_3D.*deltaT)/(Tfinal-Tinici);
meanLInfinity = sum(L_Infinity_norm.*deltaT)/(Tfinal-Tinici);
meanCEffort = sum(Control_effort.*deltaT)/(Tfinal-Tinici);
meanAbsYawErr=sum(abs(yaw_ref(n1:n)-yaw(n1:n)).*deltaT(n1:n))*180/pi/(Tfinal-Tinici);
meanVel=sum(abs(sqrt(yout(n1:end,7).^2+yout(n1:end,8).^2+yout(n1:end,9).^2)').*deltaT(n1:n))/(Tfinal-Tinici);

% Diferential control effort
% tout_d = [0; diff(tout)];
% m1_d = [0 diff(yout(:,17))'];
% m2_d = [0 diff(yout(:,18))'];
% m3_d = [0 diff(yout(:,19))'];
% m4_d = [0 diff(yout(:,20))'];
% CEff=(abs(m1_d)*tout_d + abs(m2_d)*tout_d + abs(m3_d)*tout_d + abs(m4_d)*tout_d)/Tfinal/4;

clear i n Amp_path Control_effort deltaT L1_norm L2_norm_2D L2_norm_3D L_Infinity_norm
clear long n1 nv Tinici tout_d x y z yaw yaw_ref meanL1 meanL2_2D meanLInfinity