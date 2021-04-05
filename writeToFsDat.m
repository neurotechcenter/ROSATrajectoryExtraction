function [wayptfile] = writeToFsDat(folder,name,coordinates)
%WRITETOFSDAT Summary of this function goes here
%   Detailed explanation goes here
    str='\n';
    for l=1:size(coordinates,1)
        str=[str num2str(coordinates(l,:)) '\n'];
    end
    str=[str 'info \nnumpoints ' num2str(size(coordinates,1)) '\nuseRealRAS 1'];

    wayptfile=fullfile(folder,[name '.dat']);
    fileID = fopen(wayptfile,'w');
    fprintf(fileID,str);
    fclose(fileID);

end

