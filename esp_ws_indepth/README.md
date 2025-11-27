# HERO-Alpha WHEEL MODULE
## Preparation

- VSCode & PlatformIO
    - install VSCode 
    - Open VSCode Extension Manager
    - Search for official PlatformIO IDE extension
    - Install PlatformIO IDE.
- ROS2 
    - https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html

    

## Step to run HERO-Alpha WheelModule

1. power on raspberry pi 
2. connect to hero wifi

```
ssid : hero
no password 
```

3. ssh to rpi using this command

``` b 
ssh hero-ptt@10.42.0.1
```
```
password : hero-ptt
```

4. run micro ros agent on terminal that ssh to rpi 
``` b 
docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888 -v6
```

    
5. open another terminal on computer and ssh to rpi again

6. run ros2 joystick node 
   
   ros2 launch teleoptwistjoy teleop-launch.py 
 
 
7. press the reset button on esp32 

8. then the robot should be ready to control