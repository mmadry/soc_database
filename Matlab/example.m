function example();
% Author: Marianna Madry (marianna.madry@gmail.com)
% Date: July 2012
% Example of how to use 'read_crd.m' and 'read_syncpc.m' functions. Loaded pointclouds are visualize.
% The CRD and SYNCPC format are described in the README file.

% SYNCPC format
fileSYNCPC='./testdata/mammal01_filterSegment_000.syncpc';
points=read_syncpc(fileSYNCPC);

figure(1);
hold on;
plot3(points(:,1), points(:,2), points(:,3),['.k']);
axis equal

% CRD format
fileCRD='./testdata/mammal01_filterSegment_000.crd';
[points,colors,imXY]=read_crd(fileCRD);

figure(2); 
hold on;
if (size(points,1)>5000) 
    disp('Pointcloud has more than 5000 points. Color visualization can take a few seconds.')
end
for j=1:size(points,1);
    plot3(points(j,1), points(j,2), points(j,3),'.','Color',colors(j,:),'MarkerSize',10);
end
axis equal

