data_path = "../data1-3_bagfiles/";
bag_name = "slam_data1.bag";

%% read bag
bag = rosbag(data_path + bag_name);
bagselect = select(bag, 'Topic', '/velodyne_points');
all_msgs = readMessages(bagselect);

%% ground truth
groundtruth_select = select(bag, 'Topic', '/globalmap');
groundtruth_map = readMessages(groundtruth_select);
groundtruth_map = cvt_pt_ros2mat(groundtruth_map{1, 1});
%% ground truth transform
bagselect = select(bag, 'Topic', '/gazebo/relative_dist');
true_tf = readMessages(bagselect, 'DataFormat', 'struct');


%% start ICP slam

last_pc = cvt_pt_ros2mat(all_msgs{1, 1});

first_pose = true_tf{1, 1};
x = first_pose.BaseX;
y = first_pose.BaseY;
theta = first_pose.Angle;

first_pose = [cos(theta), sin(theta), 0, 0;
    -sin(theta), cos(theta), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

pose = affine3d(first_pose);
build_map = pctransform(last_pc, pose);

pcshow(build_map);

%% save pose graph
all_tf = cell(length(all_msgs), 1);
all_tf{1, 1} = pose;

for k = 2:length(all_msgs)
    
    fprintf('frame %d / total frame %d\n', k, length(all_msgs));
    
    % find increment transform
    new_pc = cvt_pt_ros2mat(all_msgs{k, 1});
    tform = register_pointcloud(new_pc, last_pc);
    % update current pose
    pose = affine3d(tform.T * pose.T);
    all_tf{k, 1} = pose;
    
    new_pc_aligned = pctransform(new_pc, pose);
    build_map = pcmerge(new_pc_aligned, build_map, 0.01);
    build_map = pcdownsample(build_map, 'gridAverage', 0.1);

    last_pc = new_pc;
    pcshow(build_map)
     % match with global map
    if mod(k, 5) == 1
        % if 1 == 1
        tform = match_global(build_map, groundtruth_map);
        build_map = pctransform(build_map, tform);
        pose = affine3d(pose.T * tform.T);
        for i = 1:k
            all_tf{i, 1} = affine3d(all_tf{i, 1}.T * tform.T);
        end
    end
end

%% visualize result
pcshow(build_map)
hold on;

all_pose = zeros(length(all_tf), 3);

for i = 1:length(all_tf)
    p = all_tf{i, 1}.T;
    all_pose(i, 1) = p(4, 1);
    all_pose(i, 2) = p(4, 2);
end
%% visualize groundtruth_map
pcshow(groundtruth_map)
hold on;
plot(all_pose(:, 1), all_pose(:, 2));
hold on;
%%
ground_truth_pose = zeros(length(true_tf), 2);
for i = 1:length(true_tf)
    temp = true_tf{i, 1};
    ground_truth_pose(i, 1) = temp.BaseX;
    ground_truth_pose(i, 2) = temp.BaseY;
end

% plot(ground_truth_pose(:, 1), ground_truth_pose(:, 2));
plot(ground_truth_pose(:, 1) - ground_truth_pose(1, 1), ground_truth_pose(:, 2) - ground_truth_pose(1, 2));
hold on;
