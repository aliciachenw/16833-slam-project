% data_path = "../data1-3_bagfiles/";
% bag_name = "slam_data3.bag";
% 
% %% read bag
% bag = rosbag(data_path + bag_name);
% bagselect = select(bag, 'Topic', '/velodyne_points');
% all_msgs = readMessages(bagselect);
% 
% %% ground truth
% groundtruth_select = select(bag, 'Topic', '/globalmap');
% groundtruth_map = readMessages(groundtruth_select);
% groundtruth_map = cvt_pt_ros2mat(groundtruth_map{1, 1});

% 
% %% ground truth transform
% bagselect = select(bag, 'Topic', '/gazebo/relative_dist');
% true_tf = readMessages(bagselect, 'DataFormat', 'struct');
%% start ICP slam

start_tf = affine3d(eye(4));
map = cvt_pt_ros2mat(all_msgs{1, 1});

% pose = affine3d(eye(4));

first_pose = true_tf{1, 1};
x = first_pose.BaseX;
y = first_pose.BaseY;
theta = first_pose.Angle;
rot = [cos(theta) sin(theta) 0; ...
      -sin(theta) cos(theta) 0; ...
               0          0  1];
trans = [x, y, 0];
a = [cos(theta), -sin(theta), 0, 0; ...
     sin(theta), cos(theta), 0, 0; ...
    0, 0, 1, 0; ...
    x, y, 0, 1];
pose = affine3d(a);

all_tf = cell(length(all_msgs), 1);
all_tf{1, 1} = pose;

last_pc = map;

for k = 2:length(all_msgs)
    
    fprintf('frame %d / total frame %d\n', k, length(all_msgs));
    
    new_pc = cvt_pt_ros2mat(all_msgs{k, 1});
    
    new_pc = pctransform(new_pc, pose);
            
    tform = register_pointcloud(new_pc, groundtruth_map);
        
    pose = affine3d(tform.T * pose.T);
    
    all_tf{k, 1} = pose;
  
%     new_pc_aligned = pctransform(new_pc, pose);    
%     
%     map = pcmerge(new_pc_aligned, map, 0.01);
    
    last_pc = new_pc;

end

%% visualize result
% pcshow(map)
% hold on;

all_pose = zeros(length(all_tf), 3);

for i = 1:length(all_tf)
    p = all_tf{i, 1}.T;
    all_pose(i, 1) = p(4, 1);
    all_pose(i, 2) = p(4, 2);
end
plot(all_pose(:, 1), all_pose(:, 2));


%% visualize groundtruth_map
pcshow(groundtruth_map)
hold on;
plot(all_pose(:, 1), all_pose(:, 2));

%%
ground_truth_pose = zeros(length(true_tf), 2);
for i = 1:length(true_tf)
    temp = true_tf{i, 1};
    ground_truth_pose(i, 1) = temp.BaseX;
    ground_truth_pose(i, 2) = temp.BaseY;
end
% figure();
% plot(all_pose(:, 1), all_pose(:, 2));
% hold on;
% ground_truth_pose = ground_truth_pose - ground_truth_pose(1, :);
plot(ground_truth_pose(:, 1), ground_truth_pose(:, 2));
