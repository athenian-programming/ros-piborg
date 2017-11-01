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
rosrun ros_piborg local_color_picker.py --width 700
```

### Multiple Targets
```bash
rosrun ros_piborg local_multi_object.py --bgr "52, 205, 203" --draw_line  --draw_box --draw_contour --http rosborg --hsv 5 --max_objects 8 --min_pixels 500 --width 900
```