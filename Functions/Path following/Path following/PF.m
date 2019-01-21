function [ out ] = PF( in )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path Following algorithm selector %
% Tomeu Rubí                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%Inputs
Algorithm = in(13);
Adaptive = in(14);

%PATH FOLLOWING ALGORITHM
    if Adaptive == 0
        %Not adaptive
        if Algorithm == 1
            %NLGL
            out = NLGL(in(1:12));
        elseif Algorithm == 2
            %Carrot Chasing
            out = CARROT(in(1:12));
%         elseif Algorithm == 3
%             %NEW ALGORITHM
%             out = NEW_ALGORITHM_FILE(in(1:12));
        end
    elseif Adaptive == 1
        %Adaptive
        if Algorithm == 1
            %NLGL
            out = NLGL_ad_2(in(1:12));
        elseif Algorithm == 2
            %Carrot Chasing
            out = CARROT_ad_2(in(1:12));
        end
%         elseif Algorithm == 3
%             %NEW ALGORITHM
%             out = NEW_ALGORITHM_FILE(in(1:12));
    end
end

