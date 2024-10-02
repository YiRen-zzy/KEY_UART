#colcon build
cmds=(
	"ros2 run example_topic topic_publisher"
	"ros2 launch rm_serial_driver serial_driver.launch.py"
	)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.5
done
