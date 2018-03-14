% ECE 492 Team 1
% Wi18
% Keith Mills
% Generates a readable header file, containing the 
% fuzzy logic decision matrix
% using a given MATLAB Fuzzy Logic set (.fis) file

% The output decision matrix has the form
% fMatrices_"fuzzySet".h
% Where "fuzzySet" is the 3 input, 5 output set used.
% Some data processing occurs, mainly any result for the wheels
% less than 0.01 in absolute terms, is ignored. Steering angles are
% rounded.

function [dMatFile, headerFile] = generateDecisionMatrices(fuzzySet)

hFileName = strcat("fMatrices_", fuzzySet, ".h");
headerFile = fopen(hFileName, 'w');

fprintf(headerFile, '#ifndef fMatrices_%s_H\n', fuzzySet);
fprintf(headerFile, '#define fMatrices_%s_H\n', fuzzySet);

fprintf(headerFile, 'extern const float FUZZYLOOKUP[21][21][21][5];\n');

fprintf(headerFile, '#endif');

headerFile = fclose(headerFile);

% Open the file with write permissions.
fileName = strcat("fMatrices_", fuzzySet, ".c");
dMatFile = fopen(fileName, 'w');

% Declare it's headers
fprintf(dMatFile, '#include "fMatrices_%s.h"\n', fuzzySet);

% Declare the decision matrices
fprintf(dMatFile, 'const float FUZZYLOOKUP[21][21][21][5] = {\n');

% Instantiate the fuzzy set
myFuzz = readfis(fuzzySet);

% Front ratio, from -10 to 10, 21 points
% Number is multiplied by 10 later.
for fRatio = linspace(-1.0, 1.0, 21)
    fprintf(dMatFile, '{');
    
    % Rear ratio, from -10 to 10, 21 points
    % Number is multiplied by 10 later
    for rRatio = linspace(-1.0, 1.0, 21)
        fprintf(dMatFile, '{\n');
        
        % Overall ratio, -10 to 10, 21 points
        for oRatio = linspace(-10, 10, 21)
            
            % Compute the fuzzy set outputs given the inputs.
            rawValues = evalfis([fRatio * 10, rRatio * 10, oRatio], myFuzz);
            
            % Process values
            [fl, fr, rl, rr, angle] = processOutput(rawValues);
            
            % Conditional here for array formatting.
            if oRatio == 10
                fprintf(dMatFile, '{%g, %g, %g, %g, %g}\n', fl, fr, rl, rr, angle);
            else
                fprintf(dMatFile, '{%g, %g, %g, %g, %g},\n', fl, fr, rl, rr, angle);
            end
        end
        if rRatio == 1
            fprintf(dMatFile, '}');
        else
            fprintf(dMatFile, '},');
        end
    end
    if fRatio == 1
        fprintf(dMatFile, '}};\n\n');
        dMatFile = fclose(dMatFile);
    else
        fprintf(dMatFile, '},\n');
    end
end
end

% Function takes a 5-length vector containing the raw fuzzy
% set outputs and cleans the data a little.
% very small values of less than abs(x) < 0.01 are ignored
% Steering angle is rounded. 
function [fl, fr, rl, rr, angle] = processOutput(raw)

    if abs(raw(1)) < 0.01
        fl = 0;
    else
        fl = raw(1);
    end
    
    if abs(raw(2)) < 0.01
        fr = 0;
    else
        fr = raw(2);
    end
    
    if abs(raw(3)) < 0.01
        rl = 0;
    else
        rl = raw(3);
    end
    
    if abs(raw(4)) < 0.01
        rr = 0;
    else
        rr = raw(4);
    end
    
    angle = round(raw(5) / 6) * 6;
end