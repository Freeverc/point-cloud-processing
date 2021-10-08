clc;clear;
%% 

%% Input
disp("loading data... ");
mesh = stlread('data1.stl');
mesh_points = mesh.Points;
mesh_triangles = mesh.ConnectivityList;
points = load('data1.mat', '-mat').TubeDataAllmm;
% mesh2 = stlread('data2.stl');
% mesh_points2 = mesh2.Points;
% points = importdata('data2.asc');
%% pre processing
for i= 1:length(points)
    points(i, 3) = -points(i, 3);
end


%% icp
disp("performering icp... ");
point_cloud = pointCloud(points);
mesh_point_cloud = pointCloud(mesh_points);

[tform,point_cloud_transformed,rmse] = ...
pcregistericp(point_cloud,mesh_point_cloud,...
    'Metric', 'pointToPoint',...
    'Extrapolate',true,...
    'MaxIterations', 200);

t  = pctransform(point_cloud,tform);
pcwrite(point_cloud,'p.ply');
pcwrite(t,'t.ply');
%% mesh pre processing
disp("mesh pre processing... ");
face_list = zeros(length(mesh_triangles), 3);
center_list = zeros(length(mesh_triangles), 3);
for i= 1:length(mesh_triangles)
    p1 = mesh_points(mesh_triangles(i,1), :);
    p2 = mesh_points(mesh_triangles(i,2), :);
    p3 = mesh_points(mesh_triangles(i,3), :);
    
    A = [p1;p2;p3];
    center_list(i, :) = mean(A, 1);
    center_list(i, :)
    if det(A)==0
        disp("wrong face")
    else
        face_list(i, :) =[1,1,1]* inv(A);
    end
%     disp("face i : ");
%     disp(i);
%     disp(face_list(i, :));
end

%% Error
disp("Error processing... ");
p_c_t_points = point_cloud_transformed.Location;
index_list = knnsearch(center_list,p_c_t_points, 'K',10,'Distance','euclidean');
error_list = ones(length(points), 1) * 100000;
color_list = uint8(zeros(length(points), 3));
for i = 1:length(points)
    p = p_c_t_points(i, :);
    for j = 1:length(index_list(i, :))
        p1 = mesh_points(mesh_triangles(index_list(i, j), 1), :);
        p2 = mesh_points(mesh_triangles(index_list(i, j), 2), :);
        p3 = mesh_points(mesh_triangles(index_list(i, j), 3), :);
        v3 = p3 - p1;
        v2 = p2 - p1; 
        v = p - p1;
        t2 = ((v * v2')* (v3 * v3')- (v * v3') * (v2 * v3')) / ((v3 * v3')* (v2 * v2')- (v3 * v2') * (v2 * v3'));
        t3 = ((v * v2')* (v2 * v3')- (v * v3') * (v2 * v2')) / ((v2 * v3')* (v2 * v3')- (v3 * v3') * (v2 * v2'));
        if t2 >= 0 && t2 <= 1 && t3 >= 0 && t3 <= 1 && t2 + t3 <= 1
            f = face_list(index_list(i, j), :);
            d = abs(f * p') / sqrt(f * f');
        else
            s = [(p - p1) * (p - p1)',(p - p3) * (p - p3)',(p - p3) * (p - p3)'];
            d = min(s);
        end
        
        if d < error_list(i)
            error_list(i) = d;
        end
    end
%     disp("error i : ");
%     disp(i);
%     disp(error_list(i));
end
%% output
disp("writing data... ");
color_map = jet();
min_error = min(error_list)
max_error = 0.4 * max(error_list)
for i = 1:length(points)
    id = fix(255 * (error_list(i) - min_error) / (max_error - min_error)) + 1;
    if id > 256
        id = 256;
    end
    color_list(i, :) = uint8(255 * color_map(id, :));
end

point_cloud.Color = color_list;
point_cloud_transformed.Color = color_list;
pcwrite(point_cloud,'point_cloud.ply');
pcwrite(point_cloud_transformed,'point_cloud_transformed.ply');
pcshow(point_cloud_transformed);