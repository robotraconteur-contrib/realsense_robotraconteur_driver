function realsense_client_pointcloud()

if nargin == 0
    realsense_url = 'rr+tcp://localhost:25415?service=PC_Service';
end

c=RobotRaconteur.ConnectService(realsense_url);

point_cloud = c.capture_point_cloud();

scatter3(point_cloud.points(1,:),point_cloud.points(2,:),point_cloud.points(3,:));

