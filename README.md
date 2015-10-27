# ps5_kvc2

This package illustrates use of trajectory_msgs/JointTrajectory, using an action-server, action-client pair.
It illustrates how to populate a JointTrajectory message, put this message in an action-server goal, and 
send the goal request to a trajectory action server.  A simple sinusoid is used to define the example trajectory.
It is sampled coarsely, at irregular intervals (intentionally).

The corresponding server accepts and executes the goal.  The server interpolates between coarse values of trajectory points,
resulting in smoother motion than pure execution of the verbatim trajectory.

To see the effect of not interpolating, comment out the line:
        fraction_of_range = (t_stream-t_previous)/(t_next-t_previous);
in the action server.  Then, only the coarse points will be commanded, updated at the specified times.
 

## Example usage
start the minimal robot with:
`roslaunch ps5_kvc2 ps5_kvc2.launch`
This will automatically start the trajectory action server and client.

Restart the trajectory action client with:
`rosrun ps5_kvc2 ps5_kvc2_trajectory_action_client`

Can see interpolation results with rqt_plot, plotting value /pos_cmd/data
