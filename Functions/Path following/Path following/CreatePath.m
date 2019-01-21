function [ x, y, z ] = CreatePath( type, zvar, A )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the path   %
% Tomeu Rubí               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

sym gamma;
g_final=100000;
gamma = 1:1:g_final;
    switch type
        case 1
            % A=2.5;
            %Path type: 8
            x=2*A*cos(0.001*gamma);
            y=A*sin(2*0.001*gamma);
            if zvar == 1
                z=0.001*gamma+3;
            else
                z(1:g_final)=3;
            end
        case 2
            % A=3;
            %Path type: circle
            x=A*cos(0.001*gamma);
            y=A*sin(0.001*gamma);
            if zvar == 1
                z=0.001*gamma+3;
            else
                z(1:g_final)=3;
            end
        case 3
            %Path type: spiral
            % A=1.25;
            for i = 1:1:g_final
                %spiral 1
                % x(i)=A*B^(0.001*i)*cos(0.001*i);
                % y(i)=A*B^(0.001*i)*sin(0.001*i);
                %spiral 2
                x(i)=0.5*A*(0.001*i)*cos(0.001*i);
                y(i)=0.5*A*(0.001*i)*sin(0.001*i);
            end
            if zvar == 1
                z=0.001*gamma+3;
            else
                z(1:g_final)=3;
            end
        case 4
            %Path type: Line + circle
            x(1:1000)=0.01*gamma(1:1000);
            y(1:1000)=0;
            x(1001:g_final)=A*cos(0.001*(gamma(1001:g_final)-gamma(1000))-pi/2) + x(1000);
            y(1001:g_final)=A*sin(0.001*(gamma(1001:g_final)-gamma(1000))-pi/2) + A;
            if zvar == 1
                z=0.001*gamma+3;
            else
                z(1:g_final)=3;
            end
%       TO DO
%       case 5
%           %NEW PATH
%           %Fill x, y and z, for gamma = 1 to g_final

    end
    clear i

end

