% ECE 492 Team 1
% Wi18
% Keith Mills
% Generates a readable header file, containing the 
% fuzzy logic decision matrix
% using a given MATLAB Fuzzy Logic set (.fis) file

function dMatFile = generateDecisionMatrices(fuzzySet)

fileName = strcat("fMatrices_", fuzzySet, ".h");
dMatFile = fopen(fileName, 'w');

fprintf(dMatFile, '#ifndef fMatrices_%s_H\n', fuzzySet);
fprintf(dMatFile, '#define fMatrices_%s_H\n', fuzzySet);

fprintf(dMatFile, 'const float FUZZYLOOKUP[21][21][21][5] = {\n');

myFuzz = readfis(fuzzySet);

for fRatio = linspace(-1.0, 1.0, 21)
    fprintf(dMatFile, '{');
    for rRatio = linspace(-1.0, 1.0, 21)
        fprintf(dMatFile, '{\n');
        for oRatio = linspace(-1.0, 1.0, 21)
            rawValues = evalfis([fRatio, rRatio, oRatio], myFuzz);
            [fl, fr, rl, rr, angle] = processOutput(rawValues);
            if oRatio == 1
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
        fprintf(dMatFile, '}};\n\n#endif');
        dMatFile = fclose(dMatFile);
    else
        fprintf(dMatFile, '},\n');
    end
end
end
function [fl, fr, rl, rr, angle] = processOutput(raw)

    if raw(1) < 0.01
        fl = 0;
    else
        fl = raw(1);
    end
    
    if raw(2) < 0.01
        fr = 0;
    else
        fr = raw(2);
    end
    
    if raw(3) < 0.01
        rl = 0;
    else
        rl = raw(3);
    end
    
    if raw(4) < 0.01
        rr = 0;
    else
        rr = raw(4);
    end
    
    angle = round(raw(5), 0); 
end