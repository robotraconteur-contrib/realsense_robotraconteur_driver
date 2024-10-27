function realsense_client_image(realsense_url)

if nargin == 0
    realsense_url = 'rr+tcp://localhost:25415?service=Multi_Cam_Service';
end

c=RobotRaconteur.ConnectService(realsense_url);
cam=c.get_cameras(0);
rr_img=cam.capture_frame();
img=robotraconteur_image_rgb_to_matrix(rr_img);
imshow(img)

    function mat_img = robotraconteur_image_rgb_to_matrix(rrimg)
    
    b=reshape(rrimg.data(1:3:end),rrimg.image_info.width,rrimg.image_info.height)';
    g=reshape(rrimg.data(2:3:end),rrimg.image_info.width,rrimg.image_info.height)';
    r=reshape(rrimg.data(3:3:end),rrimg.image_info.width,rrimg.image_info.height)';

    mat_img=cat(3,r,g,b);
