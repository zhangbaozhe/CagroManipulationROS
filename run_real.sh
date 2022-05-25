xterm -e "source ~/.bashrc && source devel/setup.bash && roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=wx250s use_actual:=true dof:=6" & 
xterm -e "sleep 5 && source ~/.bashrc && source devel/setup.sh && roslaunch object_color_detector object_detect_hsv.launch"
xterm -3 "sleep 10 && source ~/.bashrc && source devel/setup.sh && rosrun object_color_detector main_project2.py"