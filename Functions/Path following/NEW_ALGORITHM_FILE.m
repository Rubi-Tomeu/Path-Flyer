function [ out ] = NEW_ALGORITHM_FILE( in )

%% Inputs
%Current Position
x=in(1); y=in(2); z=in(3);
%Current Yaw
psi=in(4);
%Current speed
u=in(5); v=in(6);
%Path type
path_type=in(7);
%Variable altitude (boolean)
z_var=in(8);
%Velocity reference
Vref=in(9);
%gamma end (last value of gamma)
g_end=in(10)*2*pi;
%algorithm parameter
ALG_PARAM=in(11); % Change for the parameter name of the algorithm
%Path amplitude
A=in(12);

%% Inicialitzation
%Definition of the path reference
persistent P_x P_y P_z g_final;
if isempty(P_x)
    [P_x,P_y,P_z]=CreatePath(path_type,z_var,A);
    [~,g_final]=size(P_x);
end

%% TO DO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Body of the algorithm: Fill outputs of the funtion

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Outputs
out(1)= u_cmd; out(2)= v_cmd; out(3)=psi_cmd; out(4)=z_cmd;
%Auxiliar outputs
out(5)= ErrX; out(6) = ErrY; out(7) = ErrZ; out(8) = End_Path; 
out(9) = ALG_PARAM; %This output can be used for debugging

end