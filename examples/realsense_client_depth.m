function realsense_client_depth(realsense_url)

if nargin == 0
    realsense_url = 'rr+tcp://localhost:25415?service=Multi_Cam_Service';
end

c=RobotRaconteur.ConnectService(realsense_url);
cam=c.get_cameras(1);
rr_img=cam.capture_frame();
depth_img=robotraconteur_image_depth_u16_to_matrix(rr_img);
depth_img_d=double(depth_img); % Only converted to double for display
myColorMap = jet(max(depth_img_d,[],'all'));
img=ind2rgb(depth_img_d,myColorMap);
imshow(img)

    function mat_img = robotraconteur_image_depth_u16_to_matrix(rrimg)

    mat_img=reshape(typecast(rrimg.data,'uint16'),rrimg.image_info.width,rrimg.image_info.height)';
    
