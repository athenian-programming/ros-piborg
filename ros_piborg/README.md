# ros-piborg 

Run piborg_controller with:
```bash
$ rosrun ros_piborg piborg_controller.py
```

Run process_image with:
```bash
$ rosrun ros_piborg process_image.py
```

$ rosrun ros_piborg ros_color_picker.py -x --img_topic /raspicam_node/image/compressed --compressed --http paris.local

$ rosrun ros_piborg local_color_picker.py -x --http rosborg.local

$ rosrun ros_piborg local_single_object.py -x --bgr "174, 56, 5" --http rosborg.local

roslaunch raspicam_node camerav2_1280x960.launch

Install *raspicam_node* as described [here](https://github.com/UbiquityRobotics/raspicam_node).