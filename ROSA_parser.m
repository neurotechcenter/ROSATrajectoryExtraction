fid=fopen('/Users/madamek/Box/BJH005/notes/BJH005.ros');

% ROSA trajectories are in L,P,S
RPS_to_RAS=[-1 0 0 0;
            0 -1 0 0;
            0 0 1 0;
            0 0 0 1];


trajectory_inf=struct('name',{},'info',{},'first_line',{});

TRRobRDisplay=struct([]);

while(~feof(fid))
    rline=fgetl(fid);
    if(strcmp(rline,'[ELLIPS]') || strcmp(rline,'[TRAJECTORY]'))
        trajectory_inf(end+1).first_line=fgetl(fid);
        rline=fgetl(fid);
        tokens=split(rline,' ');
        trajectory_inf(end).name=tokens{1};
        trajectory_inf(end).info=[cellfun(@str2num,tokens(2:end))];
    end
    if(strcmp(rline,'[TRRobRDisplay]') & isempty(TRRobRDisplay))
        rline=fgetl(fid);
        tokens=split(rline,' ');
         tokens(cellfun('isempty',tokens))=[];
        TRRobRDisplay.Transform=reshape([cellfun(@str2num,tokens)],4,4);
 
    end
end

%last 2 prob. not important
%% print coordinates

for i=1:length(trajectory_inf)
    str='\n';
    str=[str num2str(trajectory_inf(i).info(4:6)') '\n'];
    str=[str num2str(trajectory_inf(i).info(8:10)') '\n'];
    str=[str 'info \nnumpoints ' 2 '\nuseRealRAS 1'];


    fileID = fopen([trajectory_inf(i).name '.dat'],'w');
    fprintf(fileID,str);
    fclose(fileID);
end
