function [ out ] = PF( in )
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

