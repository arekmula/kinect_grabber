rosrun topic_tools throttle messages /camera/rgb/image_raw 5.0 hz_image_raw & rosrun topic_tools throttle messages /camera/depth/image_raw 5.0 hz_depth_image_raw & rosrun topic_tools throttle messages /camera/depth_registered/points 5.0 hz_points
