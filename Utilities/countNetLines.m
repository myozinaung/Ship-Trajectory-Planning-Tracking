function nLines = countNetLines(file)

% file = 'myfile.m';                                          % name file to read
fileChar = fileread(file);                                  % read file (char)
fileStr = strtrim(regexp(fileChar, '\n', 'split'))';        % split lines, remove leading while space
% Remove empty lines
fileStr(cellfun(@isempty, fileStr)) = []; 
% Detect contiguous comments
startIdx = cumsum(cellfun(@(x)strcmp(x,'%{'), fileStr)); 
stopIdx = cumsum(cellfun(@(x)strcmp(x,'%}'), fileStr) & startIdx>0);
contigIdx = (startIdx - stopIdx) > 0; 
fileStr(contigIdx) = []; 
% Remove lines that are comments
fileStr(cellfun(@(x)strcmp(x(1),'%'), fileStr)) = []; 
% Count number of lines left
nLines = length(fileStr);
