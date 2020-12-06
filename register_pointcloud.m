function tform = register_pointcloud(pt1, pt2)
tform = pcregistericp(pt1, pt2, 'Metric', 'pointToPlane', 'Extrapolate',true, 'MaxIterations', 100);
end