^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package global_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3.3 (1.6.3) (2019-06-24)
-------------------
* Deletion of fix_pipe_radius parameter.
* Addition of set_a_radius service.
* Addition of set_a_rightshift service.

0.1.3.2 (1.6.2) (2019-06-17)
-------------------
* Addition of pipeline visualisation function for rviz.
* Addition of a code for a robot to run in the right hand side from the
pipeline segment centre.

0.1.3.1 (1.6.1) (2019-05-29)
-------------------
* Addition of latch flag on initialised publication.

0.1.3.0 (1.6.0) (2019-05-28)
-------------------
* Addition of initialisation notification as a topic
* Addition of reception of checkpoints as a service

0.1.2.5 (1.5.5) (2019-05-10)
-------------------
* Change of output format for robot_position topic.
* Addition of inquire_segments service.

0.1.2.4 (1.5.4) (2019-04-22)
-------------------
* Modification of main function for melodic version

0.1.2.3 (1.5.3) (2019-04-19)
-------------------
* Modification of torch model

0.1.2.2 (1.5.2) (2019-04-17)
-------------------
* Bug fix about back veil in order not to go back

0.1.2.1 (1.5.1) (2019-03-28)
-------------------
* Slight bug fix for checkpoints reception
* Slight change of specification for robot_position publication
* Addition of "get_numof_checkpoints" service

0.1.2.0 (1.5.0) (2019-03-25)
-------------------
* Addition of informing robot position function

0.1.1.1 (1.4.1) (2019-03-25)
-------------------
* Bug fix for unexpected short cut

0.1.1.0 (1.4.0) (2019-03-19)
-------------------
* Addition of torch model
* Making thread number dynamic
* Addition of thread number checking
  issue: dynamic parameter for "num_threads" doesn't change
* Making DistanceFromCentre function CUDA based

0.1.0.2 (1.3.3) (2019-03-13)
-------------------
* Slight change about costmap for fast calculation

0.1.0.1 (1.3.2) (2019-03-13)
-------------------
* Modification of consuming time and addition of time consuming publishing on topic.
* Slight bug fix for SubscribeCheckpoints function.

0.1.0.0 (1.3.1) (2019-02-28)
-------------------
* Addition of charge param.

1.3.0 (2019-02-13)
-------------------
* Addition of CUDA calculation function.

1.2.0 (2019-02-12)
-------------------
* Addition of informing start and end of makePlan to tablet by a topic.

1.1.0 (2019-02-08)
-------------------
* Addition of not fix radius mode.
* Addition of use straight line mode.

1.0.0 (2018-12-12)
-------------------
* Change of README.md for public release.

0.1.4 (2018-07-26)
-------------------
* Addition of centre_weight parameter. A robot goes in the middle with the
 parameter.
* Replacement from global_planner into navfn for base planner.

0.1.3 (2018-06-27)
-------------------
* Addition of getCheckpoints as a service

0.1.2 (2018-06-13)
-------------------
* getReadStatus and getRobotStatus as a service
* Change of the default value of pipe_radius
* Addition of the crossing check routine for pipeline

0.1.1 (2018-03-27)
-------------------
* Addition of getStatus function as a service server

0.1.0 (2018-03-26)
-------------------
* Initial commit
