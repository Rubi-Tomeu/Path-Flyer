function [ out ] = CARROT_ad_2( in )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Adaptive 3D-Carrot-Chasing Algorithm %
% Tomeu Rubí                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
% global delta;
% global lookDist

% Vref = 0.5;
% delta = 0.6;
Tol = 0.08;

%% Calculate point where evaluate radius
gamma_look = g_final;
lookDist_min = 1.5544*Vreal - 0.1787*(4.991*Vreal + 0.0333) + 0.2053;
lookDist_max = 1.5544*Vreal - 0.1787*(0) + 0.2053;
lookDist_2 = 0;
if g_curr < g_final
    dist = sqrt((P_x(g_curr)-P_x(g_curr+1)).^2+(P_y(g_curr)-P_y(g_curr+1)).^2+(P_z(g_curr)-P_z(g_curr+1)).^2);
    dist_err=abs(dist-lookDist_min);
    for i = g_curr+1:1:g_final-1
        dist = dist + sqrt((P_x(i)-P_x(i+1)).^2+(P_y(i)-P_y(i+1)).^2+(P_z(i)-P_z(i+1)).^2);
        dist_err2=abs(dist-lookDist_min);
        
         % lookDist min condition
         if (dist_err2 <= Tol)
            if (dist_err2 >= dist_err)
                gamma_look = i;
                real_lookDist = dist;
            end
            dist_err = dist_err2;
         end
        
        % Calculate radius and lookdist in a point on the path
        Radius = ValueRadius(i*0.001,path_type,A);
        if Radius > 4.991*Vreal + 0.0333
            Radius = 4.991*Vreal + 0.0333;
        end
        lookDist = 1.5544*Vreal - 0.1787*Radius + 0.2053;
        % Most urgent curve condition (gamma)
        if (lookDist >= dist) && (dist >= lookDist_min)
            if lookDist >= lookDist_2
                gamma_look = i;
                real_lookDist = dist;
                lookDist_2 = lookDist;
            end
        end
        
        % lookDist_max condition
        if (dist > lookDist_max)
            break;
        end
    end
end
gamma_look=gamma_look*0.001;
% gamma=(g_curr+170)*0.001; %V=0.4->165, V=0.5->170 V=0.6->175 V=0.7->180

%% Obtain radius and Vref
% Value of Radius in a point on the path
Radius = ValueRadius(gamma_look,path_type,A);

Radius = min(Radius,20);

% Velocity reduction
Kvmax = 0.7;
Vref = min(Vref,5.529*atan(Kvmax*0.2779*Radius+0.4759)-2.017);

%% Neural Networks to calculate L
% delta = netTOT([Vreal; Radius]);
% delta = netCC2_5_2([Vreal; Radius]);
% delta = netCC3_6_3([Vreal; Radius]);

delta = net2CC3_7_3([Vreal; Radius]);

%% Calculate minimum distance position (g_curr)
dist = sqrt((P_x(g_curr)-x).^2+(P_y(g_curr)-y).^2+(P_z(g_curr)-z).^2);
for i = g_curr+1:1:g_final
    dist2 = sqrt((P_x(i)-x).^2+(P_y(i)-y).^2+(P_z(i)-z).^2);
    if (dist2 >= dist)
        g_curr = i-1;
        break;
    end
    dist = dist2;
end

%% Calculate VTP
% If not any point at delts distance, VTP is end of the path
VTP = g_final;
% pos dist min + delta
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
out(5)= ErrX; out(6) = ErrY; out(7) = ErrZ; out(8) = End_Path; out(9) = delta;

end