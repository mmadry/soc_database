function [points,colors,imXY]=read_crd(file);
% Author: Marianna Madry (marianna.madry@gmail.com)
% Date: July 2012
% Loads a XYZ pointcloud, corresponding colors and xy image coordinates (left stereo image) to Matlab

 fid = fopen(file); % load 3D point coordinates in .syncpc format
 tmp = textscan(fid,'%s', 1);
 data = cell2mat(textscan(fid,'%f %f %f %f %f %f %f %f'));
 points = data(:,1:3);
 imXY = data(:,4:5);
 colors = data(:,6:8)/255;
 fclose(fid);