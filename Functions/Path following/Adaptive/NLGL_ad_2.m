function [ out ] = NLGL_ad_2( in )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Adaptative 3D-NLGL Algorithm  %
% Tomeu Rubí                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%Current Speed
u=in(5); v=in(6); Vreal = sqrt(u^2+v^2);
%Path type
path_type=in(7);
%Variable altitude (boolean)
z_var=in(8);
%Velocity reference
Vref=in(9);
%gamma end (last value of gamma)
g_end=in(10)*2*pi;
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

% global Vref;
% global lookDist;
% global L;
% global Radi;

% Vref = 0.5;
Tol = 0.08;

%% Calculate point where evaluate radius
gamma_look = g_final;
lookDist_min = 1.5544*Vreal - 0.1787*(4.991*Vreal + 0.0333) + 0.2053;
lookDist_max = 1.5544*Vreal - 0.1787*(0) + 0.2053;
% lookDist_2 = 0;
if g_curr < g_final
    dist = sqrt((P_x(g_curr)-P_x(g_curr+1)).^2+(P_y(g_curr)-P_y(g_curr+1)).^2+(P_z(g_curr)-P_z(g_curr+1)).^2);
    dist_err=abs(dist-lookDist_min);
    Rrest = 10000;
    for i = g_curr+1:1:g_final-1
        dist = dist + sqrt((P_x(i)-P_x(i+1)).^2+(P_y(i)-P_y(i+1)).^2+(P_z(i)-P_z(i+1)).^2);
        dist_err2=abs(dist-lookDist_min);
        
        % lookDist min condition
         if (dist_err2 <= Tol)
            if (dist_err2 >= dist_err)
                gamma_look = i;
                % real_lookDist = dist;
            end
            dist_err = dist_err2;
         end
        
        % Calculate radius and lookdist in a point on the path
        Radius = ValueRadius(i*0.001,path_type,A);
        Rsat = min(Radius, 4.991*Vreal + 0.0333);
        lookDist = 1.5544*Vreal - 0.1787*Rsat + 0.2053;
        % Most urgent curve condition (gamma)
        if (lookDist >= dist) && (dist >= lookDist_min)
        % if (dist >= lookDist_min)
            if Radius < Rrest
                gamma_look = i;
                % real_lookDist = dist;
                Rrest = Radius;
            end
        end
        
        % lookDist_max condition
        if (dist > lookDist_max)
            break;
        end
    end
end
gamma_look=gamma_look*0.001;
% gamma_look=(g_curr+170)*0.001; %V=0.4->165, V=0.5->170 V=0.6->175 V=0.7->180

%% Obtain radius and Vref
% Value of Radius in a point on the path
Radius = ValueRadius(gamma_look,path_type,A);

Radius = min(Radius,20);

% Velocity reduction
Kvmax = 0.8;
Vref = min(Vref,1.886*atan(Kvmax*0.6197*Radius-1.037)+1.783);

%% Neural Networks to calculate L
% L = net(Radius);
% L = netTOT([Vreal; Radius]);

L = net2NLGL3_6_3([Vreal; Radius]);

% L = netNLGL4_4_4([Vreal; Radius]);
% L = netNLGL20([Vreal; Radius]);

%Adjust by 2 planes
% if Radius > 4.991*Vreal + 0.0333
%     L = 1.2253*Vreal + 0.0001*Radius + 0.0124;
% else
%     L = 2.0157*Vreal - 0.1858*Radius + 0.1140;
% end

%% Calculate minimum distance position (g_curr)
mindist=sqrt((P_x(g_final)-x).^2+(P_y(g_final)-y).^2+(P_z(g_final)-z).^2);

dist = sqrt((P_x(g_curr)-x).^2+(P_y(g_curr)-y).^2+(P_z(g_curr)-z).^2);
for i = g_curr+1:1:g_final
    dist2 = sqrt((P_x(i)-x).^2+(P_y(i)-y).^2+(P_z(i)-z).^2);
    if (dist2 >= dist)
        g_curr = i-1;
        mindist = dist;
        break;
    end
    dist = dist2;
end

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
u_cmd = Vref*dist2/dist3;

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