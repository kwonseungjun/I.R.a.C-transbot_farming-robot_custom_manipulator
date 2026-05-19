<launch>
    <param name="use_sim_time" value="false"/>

    <!--    Start the turtle node-->
    <node name="turtlesim_node" pkg="turtlesim" type="turtlesim_node" output="screen" respawn="false"/>

    <!--    Start the node for obtaining wireless handle information-->
    <node name="joy_node" pkg="joy" type="joy_node" output="screen" respawn="false"/>

    <!--    Start the wireless controller node-->
    <node name="twist_joy" pkg="transbot_ctrl" type="twist_joy.py" output="screen">
        <param name="linear_speed_limit" type="double" value="2.0"/>
        <param name="angular_speed_limit" type="double" value="2.0"/>
    </node>
</launch>
