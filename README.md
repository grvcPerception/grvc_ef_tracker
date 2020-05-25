# grvc_ef_tracker 
This repository includes the ROS implemenatation of the the event-based feature tracking algorithm described in the paper "**Asynchronous Event-Based Clustering and Tracking for Intrusion Monitoring in UAS**".

## Publication
In case you use this code, please cite the following [publication](): 

Juan Pablo Rodríguez-Gómez, Augusto Gómez Eguíluz, Jose Ramiro Martínez De-Dios and Anibal Ollero. "**Asynchronous Event-Based Clustering and Tracking for Intrusion Monitoring in UAS.**" IEEE International Conference on Robotics and Automation (ICRA), 2020. 

    @inproceedings{rodriguez2020asynchronous,
        title={Asynchronous Event-Based Clustering and Tracking for Intrusion Monitoring in UAS},
        author={Rodr\'{i}guez-G\'{o}mez, Juan Pablo and G\'{o}mez Egu\'{i}luz, Augusto and Martinez De-Dios, Jose Ramiro and Ollero, Anibal},
        booktitle={20120 IEEE International Conference on Robotics and Automation (ICRA)},
        year={2020},
        organization={IEEE}}

# Requirements
* [Eigen 3](https://eigen.tuxfamily.org/dox/)
* [ROS Kinetic](http://wiki.ros.org/kinetic) 
* [RPG DVS ROS](https://github.com/uzh-rpg/rpg_dvs_ros) 

# Installation
Clone the repository to your ROS workspace (e.g ~/catkin_ws/src) 


    $ git https://github.com/grvcPerception/grvc_ef_tracker.git
    $ catkin build grvc_ef_tracker
    $ source ~/catkin_ws/devel/setup.bash

The package includes a wrapper as example to use the tracker algorithm. The wrapper receives input features using the `dvs_msgs/EventArray` message format. Try the tracker by connecting your DAVIS camera and running the feature generator of your preference (e.g. corner events). Then type:

    $ roslaunch grvc_ef_tracker tracker_test.launch feature_topic:= $(feature_topic)

# License
This code is released under MIT license

We constantly work to improve algorithm performance and code reliability. For additional information please contact Juan Pablo Rodriguez Gomez <jrodriguezg@us.es>
