%% read ros message and convert to matlab pointcloud

function pt_cloud = cvt_pt_ros2mat(msg)
xyz = readXYZ(msg);

% clean unright point
index = xyz(:, 3) > -0.8;
xyz = xyz(index, :);
r = xyz(:, 1) .^ 2 + xyz(:, 2) .^ 2 + xyz(:, 3) .^ 3;
r = sqrt(r);
index = r < 15;
xyz = xyz(index, :);
pt_cloud = pointCloud(xyz);
end