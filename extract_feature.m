function [E, T] = extract_feature(pointcloud)
    cloud_curvature = zeros(pointcloud.Count, 1);
    for i = 5:pointcloud - 5
        dif = pointcloud.Location(i-5:i+5, :) - 10 * pointcloud.Location(i, :);
        cloud_curvature(i, 1) = dif(1) * dif(1) + dif(2) * dif(2) + dif(3) * dif(3);
        
    end
end