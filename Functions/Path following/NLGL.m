function [ out ] = NLGL( in )
%%%%%%%%%%%%%%%%%%%%%
% 3D-NLGL Algorithm %
% Tomeu Rubí        %
%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2019 Bartomeu Rubí, Adrián Ruiz, Ramon Pérez, Bernardo Morcego
%
% Path-Flyer is free software; you can redistribute it and/or modify it under 
% the terms of the GNU General Public License as published by the Free Software 
% Foundation; either version 3, or (at your option) any later version.
%
% Path-Flyer is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR 
% A PARTICULAR PURPOSE. See the GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License along with 
% the benchmark files. If not, see http://www.gnu.org/licenses/.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
L=in(11);
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

%NLGL variables
% L=0.59; %circle A=3
% L=0.75; %eight A=2.5
Tol=0.08;

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
mindist=sqrt((P_x(g_final)-x).^2+(P_y(g_final)-y).^2+(P_z(g_final)-z).^2);

dist = sqrt((P_x(g_curr)-x).^2+(P_y(g_curr)-y).^2+(P_z(g_curr)-z).^2);
for i = g_curr+1:1:g_final
    dist2 = sqrt((P_x(i)-x).^2+(P_y(i)-y).^2+(P_z(i)-z).^2);
    if (dist2 >= dist)
        Index = i-1;
        mindist = dist;
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
% If not any point at L distance, VTP is end of the path (if mindist <= L)
VTP = g_final;
if mindist > L
    %Still not arrive to the path
    VTP = g_curr;
else
    % pos dist L -> NLGL
    dist = sqrt((P_x(g_curr)-x).^2+(P_y(g_curr)-y).^2+(P_z(g_curr)-z).^2);
    dist_err = abs(dist-L);
    for i = g_curr+1:1:g_final
        dist = sqrt((P_x(i)-x).^2+(P_y(i)-y).^2+(P_z(i)-z).^2);
        dist_err2 = abs(dist-L);
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

%psi_cmd
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
% u_cmd = Vref*dist2*cos(psi_cmd-psi)/L;
% u_cmd = Vref*dist2/L;
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
out(5)= ErrX; out(6) = ErrY; out(7) = ErrZ; out(8) = End_Path; out(9) = L;

end