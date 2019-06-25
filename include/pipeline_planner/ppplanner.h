/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, SEAOS, Inc.
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: t-ogata@seaos.co.jp
 *********************************************************************/
#ifndef PIPELINEPLANNER_H
#define PIPELINEPLANNER_H
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
//#include <dynamic_reconfigure/IntParameter.h>
//#include <dynamic_reconfigure/Reconfigure.h>
//#include <dynamic_reconfigure/Config.h>
//#include <dynamic_reconfigure/client.h>
//#include <global_planner/planner_core.h>
#include <pipeline_planner/PipelinePlannerConfig.h>
#include <pipeline_planner/RobotPosition.h>
#include <pipeline_planner/SegmentInfo.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <pipeline_planner/GetReadStatus.h>
#include <pipeline_planner/GetRobotStatus.h>
#include <pipeline_planner/GetCheckpoints.h>
#include <pipeline_planner/GetNumofCheckpoints.h>
#include <pipeline_planner/InquireSegments.h>
#include <pipeline_planner/ReceiveCheckpoints.h>
#include <pipeline_planner/SetARadius.h>
#include <pipeline_planner/SetARightshift.h>
#include <navfn/navfn_ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/thread.hpp>

namespace pipeline_planner {

/**
 * @class PipelinePlanner
 * @brief Provides a global planner that will make a route along check points gained from topic. This planner uses global_planner.
 */
  class PipelinePlanner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief Constructor for the PipelinePlanner
       */
      PipelinePlanner();
      /**
       * @brief Constructor for the PipelinePlanner
       * @param name The name of this planner
       * @param costmap A pointer for the costmap
       */
      PipelinePlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

      /**
       * @brief Destructor for the planner
       */
      ~PipelinePlanner();

      /**
       * @brief Initialisation function for the Planner
       * @param name The name of this planner
       * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      /**
       * @brief Initialisation function for the Planner
       * @param name The name of this planner
       * @param costmap A pointer to the costmap
       * @param frame_id  The global frame of the costmap
       */
      bool initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

      /**
       * @brief Reception of checkpoints
       * @param req Checkpoints for reception
       * @param res Status of receiving checkpoints
       * @return Always true
       */
      bool ReceiveCheckpoints(pipeline_planner::ReceiveCheckpoints::Request &req,pipeline_planner::ReceiveCheckpoints::Response &res);

      /**
       * @brief substitute pipe_radius_ value when it is called from dynamic reconfigure
       */
      void setPipeRadius(float radius);
//      void setPipeRadius(float radius){
//       pipe_radius_float=radius;
//       pipe_radius_=(double)pipe_radius_float;
//      }

      void setLethal(unsigned char lethal_cost){
       lethal_cost_ = lethal_cost;
      }

      void setWeight(float centre_weight){
       centre_weight_= centre_weight;
      }

#if MODE==MODE_GPU
      void setNumThread(float num_threads){
       num_threads_= num_threads;
      }
#endif

      void setStraight(bool use_straight){
       use_straight_ = use_straight;
      }

      void setCharge(bool charge){
       charge_= charge;
      }

      void setTimeDisplay(bool time_display){
       time_display_ = time_display;
      }

      void setTorchArea(bool use_torch,float torch_area_x,float torch_area_y){
       use_torch_=use_torch;
       torch_area_x_=torch_area_x;
       torch_area_y_=torch_area_y;
      }


      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

    private:

      /**
       * @brief Callback function for dynamic reconfiguration
       * @param config The new configuration
       * @param level ORing together level values of the parameters
       */
      void reconfigureCB(pipeline_planner::PipelinePlannerConfig &config,uint32_t level);

      /**
       * @brief Get the status of reading checkpoints
       * @param req Dummy argument for input
       * @param res The status of reading checkpoints for output
       * @return Always true
       **/
      bool getReadStatus(pipeline_planner::GetReadStatus::Request &req,pipeline_planner::GetReadStatus::Response &res){
       res.read_status=read_status_.data;
       return true;
      }

      /**
       * @brief Get the status of a robot
       * @param req Dummy argument for input
       * @param res The status of a robot for output
       * @return Always true
       **/
      bool getRobotStatus(pipeline_planner::GetRobotStatus::Request &req,pipeline_planner::GetRobotStatus::Response &res){
       res.robot_status=robot_status_.data;
       return true;
      }

      /**
       * @brief Get check points
       * @param req Dummy argument for input
       * @param res The check points and the open close flag
       * @return Always true
       **/
      bool getCheckpoints(pipeline_planner::GetCheckpoints::Request &req,pipeline_planner::GetCheckpoints::Response &res){
       int i,size;
       size=check_points_.size();
       res.checkpoints.resize(size);
       for(i=0;i<size;i++){
        res.checkpoints[i]=check_points_.at(i);
       };
       if(openclose_==1){
        res.openclose=1;
       } else if(openclose_==2){
        res.openclose=2;
       } else {
        res.openclose=3;
       };
       return true;
      }

      /**
       * @brief Get the status of reading checkpoints
       * @param req Dummy argument for input
       * @param res The status of reading checkpoints for output
       * @return Always true
       **/
      bool getNumofCheckpoints(pipeline_planner::GetNumofCheckpoints::Request &req,pipeline_planner::GetNumofCheckpoints::Response &res){
       //res.num_checkpoints=read_status_.data;
       res.num_checkpoints=check_points_.size();
       return true;
      }

      /**
       * @brief Inquiring segments information
       * @param req Dummy argument for input
       * @param res The information of segments
       * @return Always true
       */
      bool InquireSegments(pipeline_planner::InquireSegments::Request &req,pipeline_planner::InquireSegments::Response &res);

      /**
       * @brief Service to set a pipeline segment radius
       * @param req ID and radius for new pipeline segment radius
       * @param res The result and the updated value for status of reading checkpoints
       * @comment
            req.ID -1: all pipeline segment
                   others: pipeline segment ID
            res.result 0: accepted normally (only normal state)
                       1: ID is not -1, 0 nor positive
                       2: The value is illegal, id est, other than 0.0 or [0.001,50].
                            0.0 means it follows pipe_radius value.
                       3: A set of checkpoints is empty
                       4: ID is larger than the number of checkpoints
       * @return always true
       */
      bool SetARadius(pipeline_planner::SetARadius::Request &req,pipeline_planner::SetARadius::Response &res);

      /**
       * @brief Service to set a right shift value for a pipeline segment
       * @param req ID and right shift value for the pipeline segment
       * @param res The result.
       * @comment
            res.result 0: accepted normally (only normal state)
                       1: ID value is not -1, 0 nor positive
                       3: A set of checkpoints is empty
                       4: ID is larger than the number of checkpoints
       * @return always true
       */
      bool SetARightshift(pipeline_planner::SetARightshift::Request &req,pipeline_planner::SetARightshift::Response &res);

      /**
       * @brief Callback function for subscription of checkpoints data from topic
       * @param checkpoints Checkpoints data from topic
       * @comment checkpoints->header.frame_id is whether the route is closed or not
       * @comment 1:closed, 2:opened
       */
      //void SubscribeCheckpoints(const geometry_msgs::PoseArray::ConstPtr& checkpoints);

      /**
       * @brief Publication of result of reading check points
       * @param result Result status
       **/
      void PublishCheckpointsResult(std_msgs::UInt32 result);

      /**
       * @brief Verification of whether the position is in the pipeline segment or not
       * @param pose The position for verification
       * @param radius The radius of the pipeline
       * @param start_edge_radius The radius for start edge circle
       * @param end_edge_radius The radius for end edge circle
       * @param tolerance The minimum value of the length of the pipeline segment
       * @param index The index of the pipeline segment
       * @return True if the position is in the pipeline segment
       **/
      bool InPipeSegment(const geometry_msgs::PoseStamped& pose,double radius,double start_edge_radius,double end_edge_radius,double tolerance,int index);

      /**
       * @brief Whether the goal point is behind the start point in a pipe segment
       * @param idx Index of the pipeline segment
       * @param start Start position
       * @param goal Goal position
       * @return True if the goal is behind the start
       */
      bool isBehindInaSegment(int idx,
       const geometry_msgs::PoseStamped& start,
       const geometry_msgs::PoseStamped& goal);

      /**
       * @brief To check whether a sequence of checkpoints has a crossing or not
       * @param endpoint The last point of checkpoints sequence. It depends on openclose flag.
       * @param check_points Received checkpoints
       * @return True if the sequence of checkpoints doesn't have a crossing
       */
      bool hasNoCross(const int endpoint,const std::vector<geometry_msgs::PoseStamped> check_points);

      /**
       * @brief To check received checkpoints whether it has a problem on costmap or not
       * @param endpoint The last point of received checkpoints sequence
       * @return values
           1:The part of the route made by the check points is off the global costmap.(error)
           2:The route includes an obstacle.(permitted)
           0:The check points are received.(accepted)
       */
      int costCheck(int endpoint);

      /**
       * @brief Calculation of cell's distance from pipe centre.
       * @param endpoint The last point of received checkpoints sequence
       * @return Always true
       */
      bool DistanceFromCentre(int endpoint);

      /**
       * @brief Initialisation of segment info
       * @return False if one of the distances is too short
       * @Comment It is supposed check points and open close flag was well
       * @ received.
       */
      bool InitSegInfo();

      /**
       * @brief To veil costmap other than that in pipeline segments
       * @param segids IDs for pipe segments to use
       * @param startindex Index for start pipe segment
       * @param goalindex Index for goal pipe segment
       * @param start Start point or current robot point
       * @return always True
       */
      bool VeilCostmap(const std::vector<int> segids,const int startindex,
             const int goalindex,const geometry_msgs::PoseStamped& start);

      /**
       * @brief To substitute plan by connecting start point, checkpoints and goal point
       * @param start The start pose
       * @param goal The goal pose
       * @param segids IDs for checkpoints to use
       * @param plan The plan, which will be filled by the function
       * @return True if valid plan is gained
       */
      bool ConnectPoints(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
                const std::vector<int> segids,std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief To substitute the plan along two points
       * @param init_point Initial point
       * @param end_point End point
       * @param plan The plan, which will be filled by the function
       * @return Always true
       */
      bool DrawOpenSegment(const geometry_msgs::PoseStamped init_point,
                const geometry_msgs::PoseStamped end_point,
                std::vector<geometry_msgs::PoseStamped>& plan);

      std::vector<double> dist_from_pipe_centre_;

      dynamic_reconfigure::Server<pipeline_planner::PipelinePlannerConfig> *dsrv_;

      /**
       * @brief To know the position of a robot in the pipeline
       * @param pose The position of the robot
       * @param ID ID number for the pipesegment to which the robot belongs
       * @param distance The distance from start check point in the pipe segment to which the robot belongs
       * @return True if the robot is in a pipesegment
       **/
      bool knowRobotPosition(const geometry_msgs::Pose pose,int& ID,
              double& distance);
      //struct used in the knowRobotPosition function
      struct asegment {
       int id; int part; double dist; bool in;
      };

      /**
       * @brief To know the position of a robot in a pipe segment
       * @param pose The pose of the robot
       * @param id The ID for a pipe segment
       * @param part Which part in a segment.
          1: in the pipe segment
          2: in the start circle
          3: in the end circle
          0: others, especially for errors or out of the pipe segment area
       * @param dist Distance to indicate the position
       * @return True if the robot is in the pipe segment
       **/
      bool whereInaPipeSegment(const geometry_msgs::Pose pose,const int id,const double radius,const double sradius,const double eradius,int& part,double &dist,double tolerance);

      /**
       * @brief To publish a path
       * @param path Path to be published
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

      /**
       * @brief To make a pipeline and publish it as a topic
       * @comment Pipeline is constructed from check_points_. Validity of
            checkpoints_ must be checked in advance.
       * @comment Publication data values
            0:in the pipeline, 100:out of the pipeline
       */
      void makePipeline();
      void makePipeline(bool legal);

      /**
       * @brief To calculate the pipeline without GPU
       */
      void calcPipeline(const int numcheckpoints,const double *stonex,const double *stoney,const bool isClose,const double *radii,const double *sradii,const double *eradii,const double origin_x,const double origin_y,const unsigned int costsizex,const unsigned int costsizey,const double resolution,int *data);

      /**
       * @brief Callback function for reception of actionlib goal
       * @param goal callback message
       */
      void callbackgoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal);

      /**
       * @brief Callback function for reception of robot position
       * @param pose callback message
       */
      void callbackRobotPose(const geometry_msgs::Pose::ConstPtr& pose);

      /**
       * @brief loop for informing the robot position by a thread
       */
      void informRobotPose();

      /**
       * @brief Display consumed time on makePlan function
       * @param time_display whether it will be displayed on terminal or not
       * @param duration Duration time
       */
      void TimeDisplay(const bool time_display,const double duration);

      /**
       * @brief To make local goal on torch model.
       * @param start The start pose
       * @param goal The goal pose
       * @param local_goal The local goal calculated by the function
       * @param err Error code
            0: no error
            1: use_torch is not used
            2: start position is out of pipeline
            3: goal position is out of pipeline
            4: the length of a pipesegment is too short
            5: unexpected error, torch area might be too narrow with respect
               to the pipeline radius
            6: start point is out of torch area
       * @return True if there is no error
       * @comment we only use this function when we use torch model
       */
      bool TakeLocalGoal(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    geometry_msgs::PoseStamped& local_goal,int& err);

      /**
       * @brief Whether the line segment crosses the torch area boundary or not
       * @param tx The centre of torch area x
       * @param ty The centre of torch area y
       * @param sx The start of the line segment x
       * @param sy The start of the line segment y
       * @param ex The end of the line segment x
       * @param ey The end of the line segment y
       * @param bx The gained corss point x
       * @param by The gained corss point y
       * @return 0: segment is in the area
                 1: start point is out of area
                 2: start is in the area and end is out of area (cross)
                 -1: error
                 -2: the length of pipe segment is too short
       */
      int CrossTorchArea(const double tx,const double ty,
             const double sx,const double sy,
             const double ex,const double ey,
             double& bx,double& by);

      /**
       * @brief Taking pipline radius in a given pipeline segment
       * @param contained_radius Contained value of the radius.
            If it is 0.0 we use pipe_radius_ as a segment radius.
       * @return Gained value
       */
      double TakeSegmentRadius(const double contained_radius);

      bool initialized_;
      std::string gained_frame_id_;
      costmap_2d::Costmap2D* costmap_;
      std::vector<geometry_msgs::PoseStamped> check_points_;
      nav_msgs::OccupancyGrid pipeline_;
      std::vector<double> segment_lengths_;
      std::vector<pipeline_planner::SegmentInfo> segment_infos_;
      //ros::Subscriber sub_checkpoints_;
      ros::Subscriber sub_robot_pose_;
      //global_planner::GlobalPlanner* global_planner_;
      navfn::NavfnROS* navfn_planner_;
      costmap_2d::Costmap2D *navfn_costmap_;
      std::string name_;
      // status of reading checkpoints
      // status is contained in read_status_.data
      // 1:not yet read, 2:reading now, 3:OK, 4:read but empty,
      // 5:route overlaps obstacle, 6:out of costmap, 7:openclose flag error,
      // 8:having a crossing, others:unknown error
      std_msgs::UInt32 read_status_;
      // status of a robot
      // 1:not yet initialised, 2:ready,
      // 3:the robot is on the way to a goal or reached the goal,
      // 4:start is out of pipeline, 5:all pipe segments include start,
      // 6:goal is out of pipeline, 7:all pipe segments include goal,
      // 8:goal is behind start in a open pipeline,
      // 9:invalid check points, 10:global planner fails to make a plan,
      // 11:goal is behind start in one pipe segment
      // 12:now calculating global path
      std_msgs::UInt32 robot_status_;
      ros::Publisher pub_result_;
      ros::Publisher pub_status_;
      ros::Publisher pub_plan_;
      ros::Publisher pub_robot_position_;
      ros::Publisher pub_initialised_;
      ros::Publisher pub_pipeline_;
      double pipe_radius_;
      //double pipe_radius_float;
      unsigned char lethal_cost_;
      double centre_weight_;
      //1:close, 2:open
      int openclose_;
      double tolerance_;
      ros::ServiceServer read_state_srv_;
      ros::ServiceServer robot_state_srv_;
      ros::ServiceServer get_checkpoints_srv_;
      ros::ServiceServer get_numof_checkpoints_srv_;
      ros::ServiceServer inquire_segments_srv_;
      ros::ServiceServer receive_checkpoints_srv_;
      ros::ServiceServer set_a_radius_srv_;
      ros::ServiceServer set_a_rightshift_srv_;
      boost::thread* thread_robot_position_;

      //taking straight line model
      bool use_straight_;
      bool charge_;
      //fixing pipe radius in each pipe segment or not
      bool fix_pipe_radius_;
      //whether displaying calculation time or not
      bool time_display_;
      bool willDisplay_;
      std_msgs::Float32 consumed_time_;
      //robot position to inform
      geometry_msgs::Pose robot_position_;
      boost::mutex mutex_robot_position_;
      //Subscription of actionlib
      ros::Publisher pub_time_;
      ros::Subscriber sub_movebase_goal_;
      //torch model
      bool use_torch_;
      double torch_area_x_,torch_area_y_;

      bool debug;

      //CUDA
#if MODE==MODE_GPU
      /**
       * @brief To veil costmap other than that in pipeline segments
       * @param segids IDs for pipe segments to use
       * @param startindex Index for start pipe segment
       * @param goalindex Index for goal pipe segment
       * @param start Start point or current robot point
       * @return always True
       */
      bool VeilCostmap_cuda(const std::vector<int> segids,const int startindex,
             const int goalindex,const geometry_msgs::PoseStamped& start);

      /**
       * @brief Calculation of cell's distance from pipe centre.
       * @param endpoint The last point of received checkpoints sequence
       * @return Always true
       */
      bool DistanceFromCentre_cuda(int endpoint);

      void ThreadNumberAdjusment(bool debug);

      int num_threads_;
#endif

  };
};
#endif 
