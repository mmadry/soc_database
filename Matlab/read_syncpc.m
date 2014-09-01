function points=read_syncpc(file)
% Author: Marianna Madry (madry@csc.kth.se)
% Date: July 2013
% Loads a XYZ pointcloud in the SYNCPC format to Matlab. The SYNCPC format is described in the README file.

 fid = fopen(file); % load 3D point coordinates in .syncpc format
 tmp = textscan(fid,'%s', 2);
 points = cell2mat(textscan(fid,'%f %f %f'));
 fclose(fid);
