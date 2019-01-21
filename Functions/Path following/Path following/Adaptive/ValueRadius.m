function [ Radius ] = ValueRadius( gamma, type, A )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtain radius of path at gamma %
% Tomeu Rubí                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

    switch type
        case 1
            % eight-shaped
            % A=2.5;
            dpd = [-2*A*sin(gamma); 2*A*cos(2*gamma); 0];
            d2pd = [-2*A*cos(gamma); -4*A*sin(2*gamma); 0];
        case 2
            % circle
            % A=3;
            dpd = [-A*sin(gamma); A*cos(gamma); 0];
            d2pd = [-A*cos(gamma); -A*sin(gamma); 0];
        case 3
            % A=1.25;
            % sspiral
    %         dpd = [A*B^gamma*(cos(gamma) + log(B)*sin(gamma)); A*B^gamma*(cos(gamma)*log(B) - sin(gamma)); 0];
    %         d2pd = [A*B^gamma*(2*cos(gamma)*log(B) + (-1 + log(B)^2)*sin(gamma)); A*B^gamma*(cos(gamma)*(-1 + log(B)^2) - 2*log(B)*sin(gamma)); 0];
            % spiral 2
            dpd = [0.5*A*(cos(gamma) - gamma*sin(gamma)); 0-5*A*(sin(gamma) + gamma*cos(gamma)); 0];
            d2pd = [-0.5*A*(2*sin(gamma) + gamma*cos(gamma)); 0.5*A*(2*cos(gamma) - gamma*sin(gamma)); 0];
        case 4
        % A=3;
        % Line and circle
            if gamma <= 1
                dpd = [10; 0; 0];
                d2pd = [0; 0; 0];
            else
                dpd = [A*cos(1 - gamma); -A*sin(1 - gamma); 0];
                d2pd = [A*sin(1 - gamma); A*cos(1 - gamma); 0];
            end
    end
    corba=norm(cross(dpd,d2pd))/norm(dpd)^3;
    Radius = 1/corba;

end

