function [ out ] = CARROT( in )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3D-Carrot-Chasing Algorithm %
% Tomeu Rubí                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Inputs
%Current Position
x=in(1); y=in(2); z=in(3);
%Current Yaw
psi=in(4);
%Path type
path_type=in(7);
%Variable altitude (boolean)
z_var=in(8);
%Velocity reference
Vref=in(9);
%gamma end (last value of gamma)
g_end=in(10)*2*pi;
%algorithm parameter
delta=in(11);
%Path amplitude
A=in(12);

%% Inicialitzation
%Definition of the path reference
persistent P_x P_y P_z g_final;
if isempty(P_x)
    [P_x,P_y,P_z]=CreatePath(path_type,z_var,A);
    [~,g_final]=size(P_x);
end

% Current gamma
persistent g_curr;
if isempty(g_curr)
    g_curr=1;
end

%CARROT CHASING Variables 
% delta=0.59; %circle A=3
% delta=0.75; %eight A=2.5
Tol=0.02;
% Vref=0.5;

%% Calculate minimum distance position (g_curr)
% %Global Minimum distance point
% mindist=10000;
%  final=g_curr + 5;
%  if final > g_final
%      final = g_final;
%  end
% pos min dist
% for i = g_curr:1:g_final
%     dist = sqrt((P_x(i)-x).^2+(P_y(i)-y).^2+(P_z(i)-z).^2);
%     if dist < mindist
%         mindist = dist;
%         g_curr = i;
%     end
% end

%First point of minimun distance
dist = sqrt((P_x(g_curr)-x).^2+(P_y(g_curr)-y).^2+(P_z(g_curr)-z).^2);
for i = g_curr+1:1:g_final
    dist2 = sqrt((P_x(i)-x).^2+(P_y(i)-y).^2+(P_z(i)-z).^2);
    if (dist2 >= dist)
        Index = i-1;
        break;
    end
    dist = dist2;
end
g_curr = Index;

% %Minimum distance with fmincon (circle)
% opt = optimset('Display','off');
% fun = @(gamma)sqrt((3*cos(gamma*0.001)-x).^2+(3*sin(gamma*0.001)-y).^2+(gamma*0.001+3-z).^2);
% [gamma, mindist] = fmincon(fun,g_curr,[],[],[],[],1,g_final,[],opt);
% ErrX = 3*cos(gamma*0.001) - x;
% ErrY = 3*sin(gamma*0.001) - y;
% ErrZ = gamma*0.001+3 - z;
% g_curr = round(gamma);

% %Minimum distance with fmincon (eight-shaped)
% opt = optimset('Display','off');
% fun = @(gamma)sqrt((5*cos(gamma*0.001)-x).^2+(2.5*sin(gamma*0.002)-y).^2+(gamma*0.001+3-z).^2);
% [gamma, mindist] = fmincon(fun,g_curr,[],[],[],[],1,g_final,[],opt);
% ErrX = 5*cos(gamma*0.001) - x;
% ErrY = 2.5*sin(gamma*0.002) - y;
% ErrZ = gamma*0.001+3 - z;
% g_curr = round(gamma);

%% Calculate VTP
% If not any point at delts distance, VTP is end of the path
VTP = g_final;
% pos min dist + delta
if g_curr < g_final
    dist = sqrt((P_x(g_curr)-P_x(g_curr+1)).^2+(P_y(g_curr)-P_y(g_curr+1)).^2+(P_z(g_curr)-P_z(g_curr+1)).^2);
    dist_err = abs(dist-delta);
    for i = g_curr+1:1:g_final-1
        dist = dist + sqrt((P_x(i)-P_x(i+1)).^2+(P_y(i)-P_y(i+1)).^2+(P_z(i)-P_z(i+1)).^2);
        dist_err2 = abs(dist-delta);
        if (dist_err2 <= Tol)
            if (dist_err2 >= dist_err)
                VTP = i-1;
                break;
            end
            dist_err = dist_err2;
        end
    end
end


%% Calculate the commands
v_cmd = 0;
z_cmd=P_z(g_curr);

%Psi_cmd
psi_cmd = atan2(P_y(VTP)-y,P_x(VTP)-x);
% Correct extra "turns" of yaw angle
n=floor((psi - psi_cmd)/(2*pi));
if n < 0
    n = n + 1;
end
psi_cmd = psi_cmd + n*2*pi;

% Nearest angle
if abs(psi - psi_cmd) > pi
    if psi_cmd > psi
        psi_cmd = psi_cmd - 2*pi;
    else
        psi_cmd = psi_cmd + 2*pi;
    end
end

%u_cmd
dist2 = sqrt((P_x(VTP)-x).^2+(P_y(VTP)-y).^2);
dist3 = sqrt((P_x(VTP)-x).^2+(P_y(VTP)-y).^2+(P_z(VTP)-z).^2);
% u_cmd = Vref*dist2*cos(psi_cmd-psi)/delta;
% u_cmd = Vref*dist2/delta;
% u_cmd = Vref*dist2*cos(psi_cmd-psi)/dist3;
u_cmd = Vref*dist2/dist3;
% if u_cmd > Vref
%     u_cmd = Vref;
% end 
% if (u_cmd < 0)
%     u_cmd = 0;
% end

%% Calculate errors
ErrX = P_x(g_curr) - x;
ErrY = P_y(g_curr) - y;
ErrZ = P_z(g_curr) - z;

%End of the path?
if path_type == 4
    gamma_final = 1000+g_end*1000;
else
    gamma_final = g_end*1000;
end
if g_curr >= round(gamma_final)
    End_Path = 1;
else
    End_Path = 0;
end

%% Outputs
out(1)= u_cmd; out(2)= v_cmd; out(3)=psi_cmd; out(4)=z_cmd;
%Auxiliar outputs
out(5)= ErrX; out(6) = ErrY; out(7) = ErrZ; out(8) = End_Path; out(9) = delta;

end