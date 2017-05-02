# RobotServer

1.机器人上运行：
    roslaunch turtlebot_bringup minimal.launch
    roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/catkin_ws/src/myturtlebotpack/map/test21.yaml

２．服务器上运行：
    roslaunch kobuki_auto_docking minimal.launch

    cd /home/workstation/PycharmProjects/trunk/RobotsServer
    python manage.py runserver 0.0.0.0:8000

    cd /home/workstation/PycharmProjects/trunk/RobotsServer/script
    python MQNode.py



fetch
    server
        roscore robot fetchcore-client pmd_driver_monitor joystick_monitor
        sudo initctl list