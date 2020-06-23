# robot-assembler
assemble robot using GUI

## Document
https://github.com/agent-system/robot_assembler/blob/master/doc/Robot_Assembler%E8%AA%AC%E6%98%8E.pdf

## Assemble robot by robot-assembler
**Make sure you have installed ROS**

1. launch robot-assembler
```
roslaunch robot_assembler robot_assembler ROBOT_NAME:=your_robot_name OUTPUT_DIR:=path_to_save
# if you want to re-start assembling add option
START_WITH:=ROBOT_NAME.roboasm.l
```
2. check your assembled robot on Rviz
```
roslaunch robot_assembler urdf_check.launch model:=/OUTPUT_DIR/ROBOT_NAME.urdf
```
3. create robot for Euslisp
at __/OUTPUT_DIR__
```
rosrun euscollada collada2eus -I ROBOT_NAME.urdf -O ROBOT_NAME.l -C ROBOT_NAME.urdf.euscollada.yaml
```
4. Gazebo
```
roslaunch robot_assembler robot_assembler_gazebo.launch model:=/OUTOUT_DIR_PATH/ROBOT_NAME.urdf
```
