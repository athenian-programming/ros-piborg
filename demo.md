https://www.youtube.com/watch?v=lfwq73D7oHg

### Joystick Teleop
```bash
# On Ubuntu
roslaunch teleop_twist_joy teleop.launch joy_dev:=/dev/input/js0 joy_config:=xd3 enable_turbo_button:=1
```

```bash
# On PiBiorg
rosrun ros_piborg piborg_controller.py
```

### Color Picker
```bash
# On Raspi with ssh -X
rosrun ros_piborg local_color_picker.py -x --width 700
```

## Race
```bash
# On Ubuntu
rosrun ros_piborg video_multi_object.py -f ~/catkin_ws/src/ros-piborg/ros_piborg/images/track.m4v --bgr "93, 211, 245" --draw_line --draw_box --draw_contour --hsv 5 --max_objects 8 --min_pixels 500 --fps 30 --width 900 --display --http paris.local 
```


### Multiple Targets
```bash
# On Raspi with ssh -X
rosrun ros_piborg local_multi_object.py -x --bgr "52, 205, 203" --draw_line  --draw_box --draw_contour --hsv 5 --max_objects 8 --min_pixels 500 --width 900 --display --http rosborg.lsocal
```
