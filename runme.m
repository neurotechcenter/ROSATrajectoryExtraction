clc;
close all;
clearvars;
addpath('NIfTI_20140122')
output_dir='RAS_data';
disp('Open ROSA .ros file');
[file,path] = uigetfile('*.ros','Select ROSA .ros file');

rosa_parsed=parseROSAfile(fullfile(path,file));

ras_projected.Trajectories=rosa_parsed.Trajectories;
ras_projected.displays={};
%% Transforming from ROSA to RAS space

rot2ras=affine_rotation(deg2rad(0),deg2rad(0),deg2rad(180));
outpath_traj=fullfile(path,output_dir,'trajectories');
if(~exist(outpath_traj,'dir'))
    mkdir(outpath_traj);
end
for i=1:length(rosa_parsed.displays)
    displays=rosa_parsed.displays(i);
    vol_path=dir(fullfile(path,[displays.volume '.img']));
    [~,name]=fileparts(vol_path.name);
    outpath=fullfile(path,output_dir,name);
    if(~exist(outpath,'dir'))
        mkdir(outpath);
    end
    if(~isempty(vol_path))
        % Converting ANALYZE to nifti ----
        info = load_nii(fullfile(vol_path.folder,vol_path.name));
        [~,nm]=fileparts(vol_path.name);
        save_nii(info, fullfile(outpath,['orig_' nm '.nii']));
        % Moving nifti 0/0/0 to center of image to match ROSA coordinate
        % space
        info = load_nii(fullfile(outpath,['orig_' nm '.nii']));
        img_size=size(info.img)/2;
        info.hdr.hist.srow_x(4)=-info.hdr.dime.pixdim(2)*img_size(1);
        info.hdr.hist.srow_y(4)=-info.hdr.dime.pixdim(3)*img_size(2);
        info.hdr.hist.srow_z(4)=-info.hdr.dime.pixdim(4)*img_size(3);
        info.hdr.hist.sform_code=1; %set sform 1 so that changes are applied later on
        %image is not yet in RAS space, so we will delete the orig_ later
        %to avoid confusion
        save_nii(info, fullfile(outpath,['orig_' nm '.nii']));
        %load nii without the resampling restrictions of the nifti package 
        info = load_untouch_nii(fullfile(outpath,['orig_' nm '.nii']));
        %calculate the correct transofmration matrix that correspond to the
        %ROSA coregistration and transform to RAS
        M=[info.hdr.hist.srow_x;info.hdr.hist.srow_y;info.hdr.hist.srow_z; 0 0 0 1];
        t_out=rot2ras*displays.ATForm*M;
        info.hdr.hist.srow_x = t_out(1,:);
        info.hdr.hist.srow_y = t_out(2,:);
        info.hdr.hist.srow_z = t_out(3,:);
        info.hdr.hist.intent_name='ROSATONI';
       
        % save the ROSA coregistered and RAS transformed nifti
        save_untouch_nii(info, fullfile(outpath,[nm '.nii']));
        ras_projected.displays{i}=fullfile(outpath,[nm '.nii']);
        delete(fullfile(outpath,['orig_' nm '.nii'])); %lets delete this file since its coordinate system might confuse someone
    end
end
%% save trajectories in RAS coordinate system
% All trajectories are in the coregistration space, so all we need to do is
% transform the trajectories into RAS space by applying rot2ras

for ii=1:length(rosa_parsed.Trajectories)
    traj_tosave=[rosa_parsed.Trajectories(ii).start 1;rosa_parsed.Trajectories(ii).end 1];
    traj_tosave=(rot2ras*traj_tosave')';
    traj_tosave=traj_tosave(:,1:3);
    ras_projected.Trajectories(ii).start=traj_tosave(1,:);
    ras_projected.Trajectories(ii).end=traj_tosave(2,:);
    writeToFsDat(outpath_traj,rosa_parsed.Trajectories(ii).name,traj_tosave);
 end


%% Coregister an input with the first image in our new RAS system and projects the trajectories accordingly
% Lastly, we want to transform the trajectories into the coordinate system
% we used elsewhere. For example, coregister to the MRI used for Freesurfer
% segmentation. We will leverage spm12 for this procedure
spm12_dir='/Applications/spm12';
if(~exist(spm12_dir,'dir'))
    disp('Please select SPM12 directory');
    spm12_dir=uigetdir(spm12_dir,'SPM12 Directory');
end
addpath(spm12_dir);


disp('Please select Reference for coregistration');
[ref_img,path_inp]=uigetfile('*.*','Reference Image for coregistration');

outpath_traj=fullfile(path,output_dir,'coregistered_trajectories');
if(~exist(outpath_traj,'dir'))
    mkdir(outpath_traj);
end

%copy the volume we will coregister to the output. To make it simple we will
%use the first ROSA volume
[fp,fn,ext]=fileparts(ras_projected.displays{1});
copyfile(fullfile(fp,[fn ext]),fullfile(outpath_traj,[fn ext]));


% Create an spm12 job
% andput data into the job structure so that spm knows how to access it
job.ref = spm_vol(fullfile(path_inp,ref_img));
job.source = spm_vol(fullfile(outpath_traj,[fn ext]));

job.eoptions.cost_fun = 'nmi';
job.eoptions.sep = [4 2];
job.eoptions.fwhm = [7 7];
job.eoptions.tol = [0.02, 0.02, 0.02, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001];
job.other = {};
%run coregistration job
x = spm_coreg(job.ref, job.source, job.eoptions);

%set transformation matrix
M = spm_matrix(x);
PO = job.source;
MM = zeros(4,4,numel(PO));
MM(:,:,1) = spm_get_space(PO.fname);
spm_get_space(PO.fname, M\MM(:,:,1));

% Transform the trajectories into the coregistered space.
for ii=1:length(ras_projected.Trajectories)
    traj_tosave=[ras_projected.Trajectories(ii).start 1;ras_projected.Trajectories(ii).end 1];
    traj_tosave=(inv(job.source.mat)*traj_tosave')';
    traj_tosave=(M\MM(:,:,1)*traj_tosave')';
    traj_tosave=traj_tosave(:,1:3);
    writeToFsDat(outpath_traj,ras_projected.Trajectories(ii).name,traj_tosave);
 end
    

