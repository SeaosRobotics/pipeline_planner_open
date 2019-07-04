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
#include <pipeline_planner/ppplanner.h>
#include <pluginlib/class_list_macros.h>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <limits>

#if MODE==MODE_GPU
#include <cuda.h>
#include <cuda_runtime.h>
#endif

PLUGINLIB_EXPORT_CLASS(pipeline_planner::PipelinePlanner, nav_core::BaseGlobalPlanner)

#if MODE==MODE_GPU
int test_cuda(int a, int b);
//CUDA version calcPipeline function
//void calcPipeline_device(const int numcheckpoints,
//const double *stonex,const double *stoney,const bool isClose,
//const double *radii,const double *sradii,const double *eradii,
//const double origin_x,const double origin_y,
//const unsigned int costsizex,const unsigned int costsizey,
//const double resolution,int *data,const int nthreads);
void calling_device(const int numstones,const double *stonex,const double *stoney,
const double startx,const double starty,
const double *radius,const double *sradius,const double *eradius,
const unsigned int costsizex,const unsigned int costsizey,const double map_resolution,
const double origin_x,const double origin_y,const double *dist_from_centre,
const double centre_weight,
unsigned char lethal,
const int min_index_x,const int max_index_x,
const int min_index_y,const int max_index_y,
unsigned char *newCost,
const int nthreads,
const bool debug);
//a function for GPU calculation called from the function
//PipelinePlanner::VeilCostmap_cuda
void DistanceFromCentreCalling(const int numstones,
const double *stonex,const double *stoney,const double *stonedev,
const unsigned int costsizex,const unsigned int costsizey,
const double map_resolution, const double origin_x,const double origin_y,
double *dist_from_centre,
const int nthreads,
const bool debug);
#endif

namespace pipeline_planner {

PipelinePlanner::PipelinePlanner():
 initialized_(NULL),costmap_(NULL),navfn_planner_(NULL),lethal_cost_(253),debug(false),tolerance_(0.001),willDisplay_(false)
{
}

PipelinePlanner::PipelinePlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id):
 initialized_(NULL),costmap_(NULL),navfn_planner_(NULL),lethal_cost_(253),debug(false),tolerance_(0.001),willDisplay_(false)
{
 //initialisation
 initialize(name,costmap,frame_id);
}

PipelinePlanner::~PipelinePlanner(){
 //if(global_planner_)delete global_planner_;
 if(navfn_planner_)delete navfn_planner_;
 if(navfn_costmap_)delete navfn_costmap_;
 if(dsrv_)delete dsrv_;
 if(thread_robot_position_)delete thread_robot_position_;
}

void PipelinePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
 bool status;
 if(!initialized_){
  status=initialize(name,costmap_ros->getCostmap(),costmap_ros->getGlobalFrameID());
  robot_status_.data=1;
  if(status){
   initialized_= true;
   robot_status_.data=2;
   std_msgs::Bool initialised;
   initialised.data=true;
   ros::Rate r1(1.0); ros::Rate r2(2.0);
   r1.sleep();
   pub_initialised_.publish(initialised);
   r2.sleep();
  } else {
   return ;
  };
 };
}

bool PipelinePlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id){
 if(initialized_){
  ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  return false;
 };

 gained_frame_id_=frame_id;
 //global_planner_=new global_planner::GlobalPlanner(name,costmap,frame_id);
 //global_planner_=new global_planner::GlobalPlanner("GlobalPlanner",costmap,frame_id);
 navfn_costmap_=new costmap_2d::Costmap2D(*costmap);
 //navfn_planner_=new navfn::NavfnROS("NavfnROS",costmap,frame_id);
 navfn_planner_=new navfn::NavfnROS("NavfnROS",navfn_costmap_,frame_id);
 costmap_=costmap;

 check_points_.clear();
 segment_infos_.clear();
 ros::NodeHandle nh1("~/"+name);
 //sub_checkpoints_=nh1.subscribe<geometry_msgs::PoseArray>
 //   ("checkpoints",1,&PipelinePlanner::SubscribeCheckpoints,this);

 willDisplay_=false;
 sub_movebase_goal_=nh1.subscribe<move_base_msgs::MoveBaseActionGoal>
    ("/move_base/goal",1,&PipelinePlanner::callbackgoal,this);

 sub_robot_pose_=nh1.subscribe<geometry_msgs::Pose>
    ("/robot_pose",1,&PipelinePlanner::callbackRobotPose,this);

 robot_position_.position.x=0.0;
 robot_position_.position.y=0.0;
 robot_position_.position.z=0.0;
 robot_position_.orientation.x=0.0;
 robot_position_.orientation.y=0.0;
 robot_position_.orientation.z=0.0;
 robot_position_.orientation.w=0.0;

 name_.clear();
 name_=name;
 read_status_.data=1;
 pub_result_=nh1.advertise<std_msgs::UInt32>("read_checkpoints",1);
 pub_status_=nh1.advertise<std_msgs::UInt32>("inform_status",1);
 pub_plan_=nh1.advertise<nav_msgs::Path>("plan",1);
 pub_robot_position_=nh1.advertise<pipeline_planner::RobotPosition>("robot_position",1);
 pub_time_=nh1.advertise<std_msgs::Float32>("time",1);
 pub_initialised_=nh1.advertise<std_msgs::Bool>("initialised",1,true);
 pub_pipeline_=nh1.advertise<nav_msgs::OccupancyGrid>("visualise_pipeline",1,true);
 PublishCheckpointsResult(read_status_);

 //nh1.param("fix_pipe_radius",fix_pipe_radius_,false);//TODO we will change default value as false

 read_state_srv_=nh1.advertiseService("get_read_status",&PipelinePlanner::getReadStatus,this);
 robot_state_srv_=nh1.advertiseService("get_robot_status",&PipelinePlanner::getRobotStatus,this);
 get_checkpoints_srv_=nh1.advertiseService("get_checkpoints",&PipelinePlanner::getCheckpoints,this);
 get_numof_checkpoints_srv_=nh1.advertiseService("get_numof_checkpoints",&PipelinePlanner::getNumofCheckpoints,this);
 inquire_segments_srv_=nh1.advertiseService("inquire_segments",&PipelinePlanner::InquireSegments,this);
 receive_checkpoints_srv_=nh1.advertiseService("receive_checkpoints",&PipelinePlanner::ReceiveCheckpoints,this);
 set_a_radius_srv_=nh1.advertiseService("set_a_radius",&PipelinePlanner::SetARadius,this);
 set_a_rightshift_srv_=nh1.advertiseService("set_a_rightshift",&PipelinePlanner::SetARightshift,this);

 dsrv_=new dynamic_reconfigure::Server<pipeline_planner::PipelinePlannerConfig>(nh1);
 dynamic_reconfigure::Server<pipeline_planner::PipelinePlannerConfig>::CallbackType cb=boost::bind(&PipelinePlanner::reconfigureCB,this,_1,_2);
 dsrv_->setCallback(cb);

 //thread
 thread_robot_position_=new boost::thread(boost::bind(&PipelinePlanner::informRobotPose,this));

 //wait for service
 for(int i=0;i<10;i++){
  bool come=ros::service::waitForService("/move_base/PipelinePlanner/receive_checkpoints",1000);
  if(come){
   ROS_INFO("Service /move_base/PipelinePlanner/receive_checkpoints has advertised.");
   break;
  } else {
  };
  if(i==9){
   ROS_ERROR("Service /move_base/PipelinePlanner/receive_checkpoints preparation is not confirmed.");
  };
 };

 makePipeline();

 return true;
}

void PipelinePlanner::reconfigureCB(pipeline_planner::PipelinePlannerConfig &config,uint32_t level){
 setPipeRadius(config.pipe_radius);
 setLethal(config.lethal_cost);
 setWeight(config.centre_weight);
#if MODE==MODE_GPU
 setNumThread(config.num_threads);
#endif
 setStraight(config.use_straight);
 setCharge(config.charge);
 setTimeDisplay(config.time_display);
 setTorchArea(config.use_torch,config.torch_area_x,config.torch_area_y);
}

void PipelinePlanner::setPipeRadius(float radius){
 pipe_radius_=(double)radius;
 if(read_status_.data==1)return; //before initialised
 read_status_.data=2;
 robot_status_.data=9;
 PublishCheckpointsResult(read_status_);
 int npoints=check_points_.size();
 int endpoint=(openclose_==1)?npoints:npoints-1; // close or open
 //costmap check
 int flag=0;
 flag=costCheck(endpoint);
 bool legal;
 //if(flag==1)pipe_radius_=-1.0;
 if(flag==1){ legal=false;} else { legal=true;};
 makePipeline(legal);
 if(flag!=1){
#if MODE==MODE_GPU
  DistanceFromCentre_cuda(endpoint);
#else
  DistanceFromCentre(endpoint);
#endif
 };

 if(flag==1){
  ROS_ERROR_STREAM("The part of the route made by the check points is off the global costmap.");
  read_status_.data=6;
  robot_status_.data=9;
  //pipe_radius_=(double)radius;
  //check_points_.clear();
  //makePipeline();
  PublishCheckpointsResult(read_status_);
  //res.read_status=read_status_.data;
 } else if(flag==2){
  ROS_INFO_STREAM("The route includes an obstacle.");
  read_status_.data=5;
  robot_status_.data=2;
  PublishCheckpointsResult(read_status_);
  //res.read_status=read_status_.data;
 } else {
  ROS_INFO_STREAM("The check points are received.");
  read_status_.data=3;
  robot_status_.data=2;
  PublishCheckpointsResult(read_status_);
  //res.read_status=read_status_.data;
 };

}

bool PipelinePlanner::SetARadius(pipeline_planner::SetARadius::Request &req,pipeline_planner::SetARadius::Response &res){
 //assertion of valid ID value
 if(!(req.ID>=-1)){
  res.result=1;
  res.read_status=read_status_.data;
  return true;
 };
 //assertion of valid radius value
 if(!(req.radius==0||req.radius>=0.001&&req.radius<=50)){
  res.result=2;
  res.read_status=read_status_.data;
  return true;
 };
 //assertion of non empty checkpoints
 int ncheckpoints=check_points_.size();
 if(check_points_.empty()){
  res.result=3;
  res.read_status=read_status_.data;
  return true;
 };
 int nsegments=(openclose_==1)?ncheckpoints:ncheckpoints-1;
 //all segments
 if(req.ID==-1){
  for(int i=0;i<nsegments;i++){
   check_points_.at(i).pose.position.z=req.radius;
  };
 } else {//each segment
  if(req.ID>nsegments-1){
   res.result=4;
   res.read_status=read_status_.data;
   return true;
  } else {
   check_points_.at(req.ID).pose.position.z=req.radius;
  };
 };
 res.result=0;
 //generation of new pipeline and validation of that
 read_status_.data=2;
 robot_status_.data=9;
 PublishCheckpointsResult(read_status_);
 int flag=0;
 flag=costCheck(nsegments);
 bool legal;
 if(flag==1){ legal=false;} else { legal=true;};
 makePipeline(legal);
 if(flag!=1){
#if MODE==MODE_GPU
  DistanceFromCentre_cuda(nsegments);
#else
  DistanceFromCentre(nsegments);
#endif
 };

 if(flag==1){
  ROS_ERROR_STREAM("The part of the route made by the check points is off the global costmap.");
  read_status_.data=6;
  robot_status_.data=9;
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
 } else if(flag==2){
  ROS_INFO_STREAM("The route includes an obstacle.");
  read_status_.data=5;
  robot_status_.data=2;
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
 } else {
  ROS_INFO_STREAM("The check points are received.");
  read_status_.data=3;
  robot_status_.data=2;
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
 };

 return true;
}

bool PipelinePlanner::SetARightshift(pipeline_planner::SetARightshift::Request &req,pipeline_planner::SetARightshift::Response &res){
 //assertion of valid ID value
 if(!(req.ID>=-1)){
  res.result=1;
  return true;
 };
 //assertion of non empty checkpoints
 int ncheckpoints=check_points_.size();
 if(check_points_.empty()){
  res.result=3;
  return true;
 };
 int nsegments=(openclose_==1)?ncheckpoints:ncheckpoints-1;
 //all segments
 if(req.ID==-1){
  for(int i=0;i<nsegments;i++){
   check_points_.at(i).pose.orientation.y=req.shift;
  };
 } else {//each segment
  if(req.ID>nsegments-1){
   res.result=4;
   return true;
  } else {
   check_points_.at(req.ID).pose.orientation.y=req.shift;
  };
 };
 res.result=0;
 if(read_status_.data==3||read_status_.data==5){
#if MODE==MODE_GPU
  DistanceFromCentre_cuda(nsegments);
#else
  DistanceFromCentre(nsegments);
#endif
 };
 return true;
}

bool PipelinePlanner::hasNoCross(const int endpoint,
            const std::vector<geometry_msgs::PoseStamped> check_points){
 bool hasCross=false;
 int npoints=check_points.size();
 double dbgPx,dbgPy,dbgQx,dbgQy,dbgRx,dbgRy,dbgSx,dbgSy;
 for(int i=0;i<endpoint;i++){ //line segment PQ
  int ci=i;
  int ni=(i==npoints-1)?0:i+1;
  double Px=check_points.at(ci).pose.position.x;
  double Py=check_points.at(ci).pose.position.y;
  double Qx=check_points.at(ni).pose.position.x;
  double Qy=check_points.at(ni).pose.position.y;
  for(int j=0;j<endpoint;j++){ //line segment RS
   if((i==j)||abs(i-j)==1||abs(i-j)==npoints-1)continue;
   int cj=j;
   int nj=(j==npoints-1)?0:j+1;
   double Rx=check_points.at(cj).pose.position.x;
   double Ry=check_points.at(cj).pose.position.y;
   double Sx=check_points.at(nj).pose.position.x;
   double Sy=check_points.at(nj).pose.position.y;
   // t(P-Q)=s(R-S)+(S-Q) for 0<=t<=1, 0<=s<=1 ... (1)
   // (1) is crossing condition
   // there are two cases, that is (P-Q)//(R-S) or not
   double PQx=Qx-Px; double PQy=Qy-Py;
   double RSx=Sx-Rx; double RSy=Sy-Ry;
   double SQx=Qx-Sx; double SQy=Qy-Sy;
   if(PQx*RSy==PQy*RSx){ // (P-Q)//(R-S)
    if(!(PQx*SQy==PQy*SQx))continue; // assertion of (P-Q)//(S-Q)
    // X={SQ,SP,RQ,RP} in PQ axis
    // minX<= 0 <= maxX (condition for overlapping)
    double SQ=SQx*PQx+SQy*PQy;
    double SP=(Px-Sx)*PQx+(Py-Sy)*PQy;
    double RQ=(Qx-Rx)*PQx+(Qy-Ry)*PQy;
    double RP=(Px-Rx)*PQx+(Py-Ry)*PQy;
    double minX=std::min(std::min(SQ,SP),std::min(RQ,RP));
    double maxX=std::max(std::max(SQ,SP),std::max(RQ,RP));
    if((minX<=0.0)&&(0.0<=maxX)){
     hasCross=true;
     dbgPx=Px;dbgPy=Py;dbgQx=Qx;dbgQy=Qy;
     dbgRx=Rx;dbgRy=Ry;dbgSx=Sx;dbgSy=Sy;
     break;
    };
   } else{ // not (P-Q)//(R-S)
    // x: cross point
    // x=tP+(1-t)Q=Q+tQP, x=sR+(1-s)S=S+sSR
    // s=((Q-S)*n)/((R-S)*n), t=((S-Q)*m)/((P-Q)*m)
    // 0<=s<=1, 0<=t<=1
    // n: vector perpendicular to P-Q, m: vector perpendicular to R-S
    double ppdPQx=-PQy; double ppdPQy=PQx; // n
    double ppdRSx=-RSy; double ppdRSy=RSx; // m
    double tmps=(SQx*ppdPQx+SQy*ppdPQy)/(-RSx*ppdPQx-RSy*ppdPQy);
    double tmpt=(-SQx*ppdRSx-SQy*ppdRSy)/(-PQx*ppdRSx-PQy*ppdRSy);
    if((0.0<=tmps)&&(tmps<=1.0)&&(0.0<=tmpt)&&(tmpt<=1.0)){
     hasCross=true;
     dbgPx=Px;dbgPy=Py;dbgQx=Qx;dbgQy=Qy;
     dbgRx=Rx;dbgRy=Ry;dbgSx=Sx;dbgSy=Sy;
     break;
    };
   };
  };
  if(hasCross)break;
 };
 if(hasCross){
  ROS_ERROR_STREAM("SubscribeCheckpoints: received pipeline has a crossing ");
  ROS_ERROR_STREAM("crossing segments");
  ROS_ERROR_STREAM("(" << dbgPx << "," << dbgPy << ")-(" << dbgQx << "," << dbgQy << ")");
  ROS_ERROR_STREAM("(" << dbgRx << "," << dbgRy << ")-(" << dbgSx << "," << dbgSy << ")");
 };
 bool hasnoCross=!hasCross;
 return hasnoCross;

}

int PipelinePlanner::costCheck(int endpoint){
 int flag=0;
 int npoints=check_points_.size();
 double map_resolution=costmap_->getResolution();
 for(int i=0;i<endpoint;i++){
  int ci=i;
  int ni=(i==npoints-1)?0:i+1;
  double pipe_radius;
  //if(fix_pipe_radius_){
  // pipe_radius=pipe_radius_;
  //} else {
  // if(check_points_.at(ci).pose.position.z==0.0){
  //  pipe_radius=pipe_radius_;
  // } else {
  //  pipe_radius=check_points_.at(ci).pose.position.z;
  // };
  //};
  pipe_radius=TakeSegmentRadius(check_points_.at(ci).pose.position.z);
  //makeing a square
  double cx=check_points_.at(ci).pose.position.x;
  double cy=check_points_.at(ci).pose.position.y;
  double nx=check_points_.at(ni).pose.position.x;
  double ny=check_points_.at(ni).pose.position.y;
  double minx=std::min(cx,nx)-pipe_radius;
  double miny=std::min(cy,ny)-pipe_radius;
  double maxx=std::max(cx,nx)+pipe_radius;
  double maxy=std::max(cy,ny)+pipe_radius;

if(debug){
 ROS_INFO_STREAM("minx: " << minx << ", miny: " << miny);
 ROS_INFO_STREAM("maxx: " << maxx << ", maxy: " << maxy);
 double origx=costmap_->getOriginX();
 double origy=costmap_->getOriginY();
 ROS_INFO_STREAM("origx: " << origx);
 ROS_INFO_STREAM("origy: " << origy);
};

  unsigned int minx_i,miny_i,maxx_i,maxy_i;
  unsigned int uix1,uiy1,uix2,uiy2;
  if(!(costmap_->worldToMap(minx,miny,uix1,uiy1))){
   ROS_ERROR_STREAM("the point (" << minx << "," << miny << ") is off the global costmap");
   flag=1;
   break;
  };
  if(!(costmap_->worldToMap(maxx,maxy,uix2,uiy2))){
   ROS_ERROR_STREAM("the point (" << maxx << "," << maxy << ") is off the global costmap");
   flag=1;
   break;
  };
  minx_i=std::min(uix1,uix2);
  miny_i=std::min(uiy1,uiy2);
  maxx_i=std::max(uix1,uix2);
  maxy_i=std::max(uiy1,uiy2);

  double normv=std::sqrt((nx-cx)*(nx-cx)+(ny-cy)*(ny-cy));
  double invnormv=1.0/normv;
  double refper=cx*(ny-cy)-cy*(nx-cx); //reference value for perpendicular component
  double difper=pipe_radius*invnormv*((ny-cy)*(ny-cy)-(-(nx-cx))*(nx-cx))
                +map_resolution*(0.5* std::abs(ny-cy)+0.5* std::abs(nx-cx));
  //reference value for parallel component
  double refpll=(0.5*(nx+cx))*(nx-cx)+(0.5*(ny+cy))*(ny-cy);
  //reference value for parallel component
  double difpll=(0.5*(nx-cx))*(nx-cx)+(0.5*(ny-cy))*(ny-cy)
               +map_resolution*(0.5*std::abs(nx-cx)+0.5*std::abs(ny-cy));

  //costmap cell loop in the square
  for(int ix=minx_i;ix<maxx_i+1;ix++){
  if((!debug)&&(flag!=0))break;
  for(int iy=miny_i;iy<maxy_i+1;iy++){
  //if(flag!=0)break;
   //whether the cell overlap the pipe segment or not
   double wx,wy; // centre of the costmap cell
   costmap_->mapToWorld(ix,iy,wx,wy);
   double curper=wx*(ny-cy)-wy*(nx-cx);
   double curpll=wx*(nx-cx)+wy*(ny-cy);
   //cost value
   unsigned char c=costmap_->getCharMap()[costmap_->getIndex(ix,iy)];
   //verification
   if((std::abs(curper-refper)<difper)&&(std::abs(curpll-refpll)<difpll)&&(c>=lethal_cost_)){
    if(debug){
     ROS_WARN_STREAM("The centre of the costmap cell:" << wx << "," << wy);
     ROS_WARN_STREAM("The costmap resolution: " << map_resolution);
    };
    flag=2;
    if(!debug)break;
   };
  };};
  //if(flag!=0)break;
 };

 return flag;
}

bool PipelinePlanner::DistanceFromCentre(int endpoint){
 int npoints=check_points_.size();
 unsigned int costsizex=costmap_->getSizeInCellsX();
 unsigned int costsizey=costmap_->getSizeInCellsY();
 unsigned int costsize=costsizex*costsizey;
 dist_from_pipe_centre_.clear();
 dist_from_pipe_centre_.resize(costsize);
 for(unsigned int xi=0;xi<costsizex;xi++){
 for(unsigned int yi=0;yi<costsizey;yi++){
  unsigned int idx=costmap_->getIndex(xi,yi);

  //We only calculate distance of a point in the pipeline
  if(pipeline_.data[idx]==100){
   dist_from_pipe_centre_.at(idx)=100.0;
   continue;
  };

  double tmpdist,mindist;
  double px,py;
  costmap_->mapToWorld(xi,yi,px,py);
  for(int i=0;i<endpoint;i++){
   int ci=i;
   int ni=(i==npoints-1)?0:i+1;
   double cx=check_points_.at(ci).pose.position.x;
   double cy=check_points_.at(ci).pose.position.y;
   double nx=check_points_.at(ni).pose.position.x;
   double ny=check_points_.at(ni).pose.position.y;
   double cnx=nx-cx;double cny=ny-cy;
   double leng_cn=sqrt(cnx*cnx+cny*cny);
   double dirx=cnx/leng_cn; double diry=cny/leng_cn;
   //directional component
   double dcomp_c=cx*dirx+cy*diry;
   double dcomp_n=nx*dirx+ny*diry;
   double dcomp_p=px*dirx+py*diry;
   //perpendicular component
   double ppcx=ny-cy; double ppcy=-(nx-cx);
   double leng_ppc=sqrt(ppcx*ppcx+ppcy*ppcy);
   double perpx=ppcx/leng_ppc; double perpy=ppcy/leng_ppc;
   //double tmpdist;
   if(dcomp_p>dcomp_n){
    //double npx=px-nx;double npy=py-ny;
    double npx=px-(nx+perpx*check_points_.at(i).pose.orientation.y);
    double npy=py-(nx+perpy*check_points_.at(i).pose.orientation.y);
    tmpdist=sqrt(npx*npx+npy*npy);
   } else if(dcomp_c>dcomp_p){
    //double cpx=px-cx;double cpy=py-cy;
    double cpx=px-(cx+perpx*check_points_.at(i).pose.orientation.y);
    double cpy=py-(cy+perpy*check_points_.at(i).pose.orientation.y);
    tmpdist=sqrt(cpx*cpx+cpy*cpy);
   } else {
//    //perpendicular component
//    double ppcx=ny-cy; double ppcy=-(nx-cx);
//    double leng_ppc=sqrt(ppcx*ppcx+ppcy*ppcy);
//    double perpx=ppcx/leng_ppc; double perpy=ppcy/leng_ppc;
    //double pcomp_c=cx*perpx+cy*perpy;
    double pcomp_c=cx*perpx+cy*perpy+check_points_.at(i).pose.orientation.y;
    double pcomp_p=px*perpx+py*perpy;
    tmpdist=fabs(pcomp_c-pcomp_p);
   };
   if(i==0){
    mindist=tmpdist;
   } else {
    if(tmpdist<mindist)mindist=tmpdist;
   };
  };
  dist_from_pipe_centre_.at(idx)=mindist;
 };};

 return true;
}

#if MODE==MODE_GPU
bool PipelinePlanner::DistanceFromCentre_cuda(int endpoint){
 //error task
 ThreadNumberAdjusment(false);

 //preparation
 cudaError_t error_allocation;
 unsigned int costsizex=costmap_->getSizeInCellsX();
 unsigned int costsizey=costmap_->getSizeInCellsY();
 unsigned int costsize=costsizex*costsizey;
//void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
//{
//  wx = origin_x_ + (mx + 0.5) * resolution_;
//  wy = origin_y_ + (my + 0.5) * resolution_;
//}
 double origin_x=costmap_->getOriginX();
 double origin_y=costmap_->getOriginY();
 double map_resolution=costmap_->getResolution();
 //coordinates for checkpoints, allocation for device and copy values to them.
 double *stonex,*stoney;
 double *stonedev;//deviation
 if(debug){
  error_allocation=cudaMallocManaged(&stonex,(endpoint+1)*sizeof(double));
  ROS_INFO_STREAM("allocation stonex: " << error_allocation);//for debug
  error_allocation=cudaMallocManaged(&stoney,(endpoint+1)*sizeof(double));
  ROS_INFO_STREAM("allocation stonex: " << error_allocation);//for debug
  error_allocation=cudaMallocManaged(&stonedev,(endpoint+1)*sizeof(double));
  ROS_INFO_STREAM("allocation stonedev: " << error_allocation);//for debug
 } else {
  cudaMallocManaged(&stonex,(endpoint+1)*sizeof(double));
  cudaMallocManaged(&stoney,(endpoint+1)*sizeof(double));
  cudaMallocManaged(&stonedev,(endpoint+1)*sizeof(double));
 };
 int npoints=check_points_.size();
 for(int i=0;i<endpoint;i++){
  int ci=i;
  int ni=(i==npoints-1)?0:i+1;
  stonex[i]=check_points_.at(i).pose.position.x;
  stoney[i]=check_points_.at(i).pose.position.y;
  stonedev[i]=check_points_.at(i).pose.orientation.y;
  if(i==endpoint-1){
   int lastindex=(i==npoints-1)?0:i+1;
   stonex[i+1]=check_points_.at(lastindex).pose.position.x;
   stoney[i+1]=check_points_.at(lastindex).pose.position.y;
   stonedev[i+1]=check_points_.at(lastindex).pose.orientation.y;
  };
 };
 // allocation of device variable which corresponds dist_from_pipe_centre_
 double *dist_from_centre;
 if(debug){
  error_allocation=cudaMallocManaged(&dist_from_centre,(costsize)*sizeof(double));
  ROS_INFO_STREAM("allocation dist_from_centre: " << error_allocation);
 } else {
  cudaMallocManaged(&dist_from_centre,(costsize)*sizeof(double));
 };

 //We only calculate distance of a point in the pipeline
 for(int i=0;i<costsize;i++){
  if(pipeline_.data[i]==100){
   dist_from_centre[i]=100.0;//out of the pipeline
  } else {
   dist_from_centre[i]=0.0;//in the pipeline
  };
 };

 //calling device function
 DistanceFromCentreCalling(endpoint,stonex,stoney,stonedev,costsizex,costsizey,
    map_resolution,origin_x,origin_y,dist_from_centre,
    num_threads_,debug);


 //finalisation
 //copying of variables
 dist_from_pipe_centre_.clear();
 dist_from_pipe_centre_.resize(costsize);
 for(int xi=0;xi<costsizex;xi++){
 for(int yi=0;yi<costsizey;yi++){
  int index=yi*costsizex+xi;
  dist_from_pipe_centre_.at(index)=dist_from_centre[index];
 };};
 //free allocated variables
 cudaError_t error_free;
 if(debug){
  error_free=cudaFree(stonex);
  ROS_INFO_STREAM("free stonex: " << error_free);
  error_free=cudaFree(stoney);
  ROS_INFO_STREAM("free stoney: " << error_free);
  error_free=cudaFree(stonedev);
  ROS_INFO_STREAM("free stonedev: " << error_free);
  error_free=cudaFree(dist_from_centre);
  ROS_INFO_STREAM("free dist_from_centre: " << error_free);
 } else {
  cudaFree(stonex);
  cudaFree(stoney);
  cudaFree(stonedev);
  cudaFree(dist_from_centre);
 };

}
#endif

bool PipelinePlanner::InitSegInfo(){
 int n=check_points_.size();
 int last=(openclose_==1)?n:n-1;

 for(int i=0;i<last;i++){
  pipeline_planner::SegmentInfo si;
  int id,previd,nextid;
  id=i;
  if((openclose_==2)&&(i==0)){
   previd=-1;
  } else {
   previd=(id-1+n)%n;
  };
  if((openclose_==2)&&(i==last-1)){
   nextid=-1;
  } else {
   nextid=(id+1)%n;
  };

  int cID=i;
  int nID=(i+1)%n;
  double cx=check_points_.at(cID).pose.position.x;
  double cy=check_points_.at(cID).pose.position.y;
  double nx=check_points_.at(nID).pose.position.x;
  double ny=check_points_.at(nID).pose.position.y;
  double dist=sqrt(pow(nx-cx,2.0)+pow(ny-cy,2.0));
  if(dist<tolerance_){
   ROS_ERROR_STREAM("two check points " << cID << "(" << cx << "," << cy << ")-(" << nx << "," << ny << ") are too close");
   ROS_ERROR_STREAM(" distance: " << dist);
   segment_infos_.clear();
   return false;
  };

  si.ID=id; si.prevID=previd; si.nextID=nextid;
  si.length=dist;
  si.start=false; si.end=false;

  segment_infos_.push_back(si);
 };

 return true;
}

void PipelinePlanner::PublishCheckpointsResult(std_msgs::UInt32 result){
 pub_result_.publish(result);
}

bool PipelinePlanner::InquireSegments(pipeline_planner::InquireSegments::Request &req,pipeline_planner::InquireSegments::Response &res){
 res.num_segments=segment_infos_.size();
 for(std::vector<pipeline_planner::SegmentInfo>::iterator it=segment_infos_.begin();it!=segment_infos_.end();it++){
  pipeline_planner::SegmentInfo segmentinfo;
  segmentinfo.ID=it->ID;
  segmentinfo.prevID=it->prevID;
  segmentinfo.nextID=it->nextID;
  segmentinfo.length=it->length;
  segmentinfo.start=it->start;
  segmentinfo.end=it->end;

  res.sinfo.push_back(segmentinfo);
 };
 return true;
}

bool PipelinePlanner::ReceiveCheckpoints(pipeline_planner::ReceiveCheckpoints::Request &req,pipeline_planner::ReceiveCheckpoints::Response &res){
 check_points_.clear();
 segment_infos_.clear();
 makePipeline();

 read_status_.data=2;
 robot_status_.data=9;
 PublishCheckpointsResult(read_status_);
 res.read_status=read_status_.data;
 if(req.checkpoints.poses.empty()){
  ROS_ERROR_STREAM("ReceiveCheckpoints: received checkpoint is empty");
  read_status_.data=4;
  robot_status_.data=9;
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
  return true;
 };

 int num_points=req.checkpoints.poses.size();
 for(int i=0;i<num_points;i++){
  geometry_msgs::PoseStamped tmpStm;
  tmpStm.header.frame_id=gained_frame_id_;
  tmpStm.pose.position.x = req.checkpoints.poses[i].position.x;
  tmpStm.pose.position.y = req.checkpoints.poses[i].position.y;
//  if(fix_pipe_radius_){
//   tmpStm.pose.position.z = 0;
//  } else {
//   tmpStm.pose.position.z = req.checkpoints.poses[i].position.z;
////   //assertion of validity
////   if(!(tmpStm.pose.position.z>0)){
////    ROS_ERROR_STREAM("SubscribeCheckpoints: a radius of " << i+1 << "-th check point is not valid");
////    ROS_ERROR_STREAM("radius: " << tmpStm.pose.position.z);
////    read_status_.data=9;
////    robot_status_.data=9;
////    PublishCheckpointsResult(read_status_);
////    res.read_status=read_status_.data;
////    return true;
////   };
//   //assertion of validity
//   if(!(tmpStm.pose.position.z>=0.001&&tmpStm.pose.position.z<=50)){
//    tmpStm.pose.position.z=0.0;// This means the pipe_radius value.
//   };
//  };

  double aradius=req.checkpoints.poses[i].position.z;
  if(!(aradius==0.0||aradius>=0.001&&aradius<=50)){
    ROS_ERROR_STREAM("ReceiveCheckpoints: a radius of " << i+1 << "-th check point is not valid");
    ROS_ERROR_STREAM("radius: " << tmpStm.pose.position.z);
    read_status_.data=9;
    robot_status_.data=9;
    PublishCheckpointsResult(read_status_);
    res.read_status=read_status_.data;
    return true;
  } else {
   tmpStm.pose.position.z=aradius;
  };

  tmpStm.pose.orientation.x = 0;
  tmpStm.pose.orientation.y = req.checkpoints.poses[i].orientation.y;
  tmpStm.pose.orientation.z = 0;
  tmpStm.pose.orientation.w = 1;
  check_points_.push_back(tmpStm);
 };

 //status check
 //empty
 if(check_points_.empty()){
  ROS_ERROR_STREAM("ReceiveCheckpoints: read of checkpoints failed");
  read_status_.data=4;
  robot_status_.data=9;
  check_points_.clear();
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
  return true;
 };
 //route overlaps an obstacle or a part of route is out of costmap
 int npoints=check_points_.size();
 double map_resolution=costmap_->getResolution();
 int endpoint;
 int openclose=std::atoi(req.checkpoints.header.frame_id.c_str());
 openclose_=openclose;
if(debug){
 ROS_INFO_STREAM("req.checkpoints.header.frame_id: " << req.checkpoints.header.frame_id);
 ROS_INFO_STREAM("openclose_: " << openclose_);
};
 if(!((openclose_==1)||(openclose_==2))){
  ROS_ERROR_STREAM("The value for openclose flag is starnge.: " << req.checkpoints.header.frame_id);
  ROS_ERROR_STREAM("It must be 1 or 2");
  read_status_.data=7;
  robot_status_.data=9;
  check_points_.clear();
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
  return true;
 };
 if(((openclose_==1)&&(check_points_.size()<3))||(check_points_.size()<2)){
  ROS_ERROR_STREAM("ReceiveCheckpoints: The number of check points is " << check_points_.size());
  if(openclose_==1){
   ROS_ERROR_STREAM("     Checkpoints must be 3 or more");
  } else {
   ROS_ERROR_STREAM("     Checkpoints must be 2 or more");
  };
  read_status_.data=4;
  robot_status_.data=9;
  check_points_.clear();
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
  return true;
 };
 //dump
 ROS_INFO_STREAM("checkpoints from service");
 //if(fix_pipe_radius_){
 // ROS_INFO_STREAM("(x,y)");
 // for(std::vector<geometry_msgs::PoseStamped>::iterator it=check_points_.begin();
 //   it!=check_points_.end();it++){
 //  ROS_INFO("(%f , %f)",(*it).pose.position.x, (*it).pose.position.y);
 // };
 //} else {
 // ROS_INFO_STREAM("(x,y): pipe_radius");
 // for(std::vector<geometry_msgs::PoseStamped>::iterator it=check_points_.begin();
 //   it!=check_points_.end();it++){
 //  double radius;
 //  if((*it).pose.position.z==0.0){
 //   radius=pipe_radius_;
 //  } else {
 //   radius=(*it).pose.position.z;
 //  };
 //  ROS_INFO("(%f , %f) : %f",(*it).pose.position.x, (*it).pose.position.y, radius);
 // };
 //};

 ROS_INFO_STREAM("(x,y): pipe_radius");
 for(std::vector<geometry_msgs::PoseStamped>::iterator it=check_points_.begin();
   it!=check_points_.end();it++){
  double radius;
  if((*it).pose.position.z==0.0){
   radius=pipe_radius_;
  } else {
   radius=(*it).pose.position.z;
  };
  ROS_INFO("(%f , %f) : %f",(*it).pose.position.x, (*it).pose.position.y, radius);
 };

 endpoint=(openclose_==1)?npoints:npoints-1; // close or open

 //calculation of pipe segments lengths
 if(!InitSegInfo()){
  ROS_ERROR_STREAM("some sequential checkpoints are too close.");
  read_status_.data=10;
  robot_status_.data=9;
  check_points_.clear();
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
  return true;
 };

 //assertion for pipeline crossing
 if(!hasNoCross(endpoint,check_points_)){
  read_status_.data=8;
  robot_status_.data=9;
  check_points_.clear();
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
  return true;
 };

 //costmap check
 int flag=0;
 flag=costCheck(endpoint);

 //calculation of distance from centre
 if(flag!=1){
  makePipeline();

#if MODE==MODE_GPU
  DistanceFromCentre_cuda(endpoint);
#else
  DistanceFromCentre(endpoint);
#endif
 };

 if(flag==1){
  ROS_ERROR_STREAM("The part of the route made by the check points is off the global costmap.");
  read_status_.data=6;
  robot_status_.data=9;
  //check_points_.clear();
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
 } else if(flag==2){
  ROS_INFO_STREAM("The route includes an obstacle.");
  read_status_.data=5;
  robot_status_.data=2;
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
 } else {
  ROS_INFO_STREAM("The check points are received.");
  read_status_.data=3;
  robot_status_.data=2;
  PublishCheckpointsResult(read_status_);
  res.read_status=read_status_.data;
 };

#if MODE==MODE_GPU
test_cuda(2, 3);
#endif
 return true;
}

bool PipelinePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan){
 //for debug
 ros::Time begin=ros::Time::now();
 //to inform start of calculation into tablet through RMC
 robot_status_.data=12;
 pub_status_.publish(robot_status_);
 if(!initialized_){
  ROS_ERROR("This planner has not been initialized yet, but it is being used,"
            " please call initialize() before use");
  robot_status_.data=1;
  pub_status_.publish(robot_status_);
  return false;
 };
 if(!((read_status_.data==3)||(read_status_.data==5))){
  ROS_ERROR_STREAM("The planner didn't receive valid check points.");
  ROS_ERROR_STREAM("read_status_: " << read_status_.data);
  robot_status_.data=9;
  pub_status_.publish(robot_status_);
  return false;
 };

 //verification of whether start and goal is in the pipeline or not
 bool inapipe=false;
 int lastpoint;
 lastpoint=(openclose_==1)?check_points_.size():check_points_.size()-1;
 std::vector<int> startids;
 startids.clear();
 for(int i=0;i<lastpoint;i++){
  int ci,ni,pi;
  bool inasegment;
  ci=i; ni=(i==check_points_.size()-1)?0:ci+1;
  double pipe_radius,sradius,eradius;
  //if(fix_pipe_radius_){
  // pipe_radius=pipe_radius_;
  // inasegment=InPipeSegment(start,pipe_radius,pipe_radius,pipe_radius,tolerance_,ci);
  //} else {
   pi=(ci==0)?check_points_.size()-1:ci-1;
   double crad=TakeSegmentRadius(check_points_.at(ci).pose.position.z);
   double prad=TakeSegmentRadius(check_points_.at(pi).pose.position.z);
   double nrad=TakeSegmentRadius(check_points_.at(ni).pose.position.z);
   //pipe_radius=check_points_.at(ci).pose.position.z;
   pipe_radius=crad;
   if(openclose_==2&&ci==0){
    sradius=pipe_radius;
   } else {
    //sradius=std::min(pipe_radius,check_points_.at(pi).pose.position.z);
    sradius=std::min(pipe_radius,prad);
   };
   if(openclose_==2&&ci==check_points_.size()-2){
    eradius=pipe_radius;
   } else {
    //eradius=std::min(pipe_radius,check_points_.at(ni).pose.position.z);
    eradius=std::min(pipe_radius,nrad);
   };
   inasegment=InPipeSegment(start,pipe_radius,sradius,eradius,tolerance_,ci);
  //};
  if(inasegment){
   startids.push_back(ci);
   inapipe=true;
  };
 };
 if(!inapipe){
  robot_status_.data=4;
  ROS_ERROR_STREAM("start point is out of pipeline");
  ROS_ERROR_STREAM("start point: " << start.pose.position.x << "," << start.pose.position.y);
  pub_status_.publish(robot_status_);
  return false;
 };
 if(startids.size()>=check_points_.size()){
  robot_status_.data=5;
  ROS_ERROR_STREAM("All pipe segments include start point. It is strange.");
  pub_status_.publish(robot_status_);
  return false;
 };
 inapipe=false;
 std::vector<int> goalids;
 goalids.clear();
 for(int i=0;i<lastpoint;i++){
  int ci,ni,pi;
  bool inasegment;
  ci=i; ni=(i==check_points_.size()-1)?0:ci+1;
  double pipe_radius,sradius,eradius;
  //if(fix_pipe_radius_){
  // pipe_radius=pipe_radius_;
  // inasegment=InPipeSegment(goal,pipe_radius,pipe_radius,pipe_radius,tolerance_,ci);
  //} else {
   pi=(ci==0)?check_points_.size()-1:ci-1;
   double crad=TakeSegmentRadius(check_points_.at(ci).pose.position.z);
   double prad=TakeSegmentRadius(check_points_.at(pi).pose.position.z);
   double nrad=TakeSegmentRadius(check_points_.at(ni).pose.position.z);
   //pipe_radius=check_points_.at(ci).pose.position.z;
   pipe_radius=crad;
   if(openclose_==2&&ci==0){
    sradius=pipe_radius;
   } else {
    //sradius=std::min(pipe_radius,check_points_.at(pi).pose.position.z);
    sradius=std::min(pipe_radius,prad);
   };
   if(openclose_==2&&ci==check_points_.size()-2){
    eradius=pipe_radius;
   } else {
    //eradius=std::min(pipe_radius,check_points_.at(ni).pose.position.z);
    eradius=std::min(pipe_radius,nrad);
   };
   inasegment=InPipeSegment(goal,pipe_radius,sradius,eradius,tolerance_,ci);
  //};
  if(inasegment){
   goalids.push_back(ci);
   inapipe=true;
  };
 };
 if(!inapipe){
  robot_status_.data=6;
  ROS_ERROR_STREAM("goal point is out of pipeline");
  ROS_ERROR_STREAM("goal point: " << goal.pose.position.x << "," << goal.pose.position.y);
  pub_status_.publish(robot_status_);
  return false;
 };
 if(goalids.size()>=check_points_.size()){
  robot_status_.data=7;
  ROS_ERROR_STREAM("All pipe segments include goal point. It is strange.");
  pub_status_.publish(robot_status_);
  return false;
 };

if(debug){
ROS_INFO_STREAM("indices which include start position");
for(int i=0;i<startids.size();i++){
int idx=startids.at(i);
double x=check_points_.at(idx).pose.position.x;
double y=check_points_.at(idx).pose.position.y;
ROS_INFO_STREAM(idx << ": " << x << "," << y);
};

ROS_INFO_STREAM("indices which include goal position");
for(int i=0;i<goalids.size();i++){
int idx=goalids.at(i);
double x=check_points_.at(idx).pose.position.x;
double y=check_points_.at(idx).pose.position.y;
ROS_INFO_STREAM(idx << ": " << x << "," << y);
};
};

 //start position and goal position
 int startindex;
 int nextindex,loopsize;
 startindex=startids.at(0);
 loopsize=startids.size();
 for(int i=0;i<loopsize;i++){
  bool next=false;
  if(!use_straight_){
   nextindex=(startindex==0)?check_points_.size()-1:startindex-1;
  } else {
   nextindex=(startindex==check_points_.size()-1)?0:startindex+1;
  };
  for(int j=0;j<loopsize;j++){
   if(startids.at(j)==nextindex){
    next=true; break;
   };
  };
  if(next){
   startindex=nextindex;
  } else {
   break;
  };
 };
if(debug){
ROS_INFO_STREAM("startindex: " << startindex);
}
 int goalindex;
 goalindex=goalids.at(0);
 loopsize=goalids.size();
 for(int i=0;i<loopsize;i++){
  bool next=false;
  nextindex=(goalindex==check_points_.size()-1)?0:goalindex+1;
  for(int j=0;j<loopsize;j++){
   if(goalids.at(j)==nextindex){
    next=true; break;
   };
  };
  if(next){
   goalindex=nextindex;
  } else {
   break;
  };
 };
if(debug){
ROS_INFO_STREAM("goalindex: " << goalindex);
};
 if((openclose_==2)&&(goalindex<startindex)){
  robot_status_.data=8;
  ROS_ERROR_STREAM("The goal point is behind a back position of the start point in the open route.");
  pub_status_.publish(robot_status_);
  return false;
 };
 //avoidance to go backwards in one pipe segment
 if(startindex==goalindex){
  if(isBehindInaSegment(startindex,start,goal)){
   robot_status_.data=11;
   ROS_ERROR_STREAM("The goal point is behind a back position of the start point in a pipe segment.");
   pub_status_.publish(robot_status_);
   return false;
  };
 };

 //selection of pipeline segments to use
 std::vector<int> segids;
 segids.clear();
 if(goalindex>=startindex){
  for(int i=startindex;i<=goalindex;i++){
   segids.push_back(i);
  };
 } else {
  for(int i=startindex;i<check_points_.size();i++){
   segids.push_back(i);
  };
  for(int i=0;i<=goalindex;i++){
   segids.push_back(i);
  };
 };
if(debug){
ROS_INFO_STREAM("segments to use");
for(int i=0;i<segids.size();i++){
 int tmp01=segids.at(i);
 double d01x=check_points_.at(tmp01).pose.position.x;
 double d01y=check_points_.at(tmp01).pose.position.y;
 ROS_INFO_STREAM(tmp01 << ":" << d01x << "," << d01y);
};
};

 if(use_straight_){
  if(!(ConnectPoints(start,goal,segids,plan))){
   robot_status_.data=10;
   plan.clear();
   pub_status_.publish(robot_status_);
   //double duration=ros::Time::now().toSec() - begin.toSec();
   //ROS_INFO_STREAM("Pipeline planner: makePlan duration time: " << duration);
   return false;
  } else {
   robot_status_.data=3;
   pub_status_.publish(robot_status_);
   publishPlan(plan);
   double duration=ros::Time::now().toSec() - begin.toSec();
   if(willDisplay_){
    TimeDisplay(time_display_,duration);
    willDisplay_=false;
   };
   return true;
  };
 };

 //veiling of the space other than the pipeline segments
#if MODE==MODE_GPU
 VeilCostmap_cuda(segids,startindex,goalindex,start);
#else
 VeilCostmap(segids,startindex,goalindex,start);
#endif

 geometry_msgs::PoseStamped local_global_goal;
 int err;
 if(use_torch_){
  if(!TakeLocalGoal(start,goal,local_global_goal,err)){
   robot_status_.data=13;
   pub_status_.publish(robot_status_);
   return false;
  };
 };

 //calculation of the plan using global_planner
 //bool resultGlobalPlanner=global_planner_->makePlan(start,goal,plan);
 bool resultGlobalPlanner;
 if(use_torch_){
  resultGlobalPlanner=navfn_planner_->makePlan(start,local_global_goal,plan);
 } else {
  resultGlobalPlanner=navfn_planner_->makePlan(start,goal,plan);
 };
 if(resultGlobalPlanner==true){
  robot_status_.data=3;
  publishPlan(plan);
 } else if(resultGlobalPlanner==false){
  robot_status_.data=10;
 };

 pub_status_.publish(robot_status_);
 //for debug
 double duration=ros::Time::now().toSec() - begin.toSec();
 if(willDisplay_){
  TimeDisplay(time_display_,duration);
  willDisplay_=false;
 };
 return resultGlobalPlanner;

}

bool PipelinePlanner::InPipeSegment(const geometry_msgs::PoseStamped& pose,
       double radius,double sradius,double eradius,double tolerance,int index){
 bool result; result=false;
 int ci,ni;
 ci=index; ni=(index==check_points_.size()-1)?0:ci+1;
 double cx=check_points_.at(ci).pose.position.x;
 double cy=check_points_.at(ci).pose.position.y;
 double nx=check_points_.at(ni).pose.position.x;
 double ny=check_points_.at(ni).pose.position.y;
 double px=pose.pose.position.x;
 double py=pose.pose.position.y;
 double normv=sqrt((nx-cx)*(nx-cx)+(ny-cy)*(ny-cy));
 if(normv<tolerance){
  ROS_ERROR_STREAM("The length of pipeline segment is too small."
                   " length: " << normv);
  return false;
 };
 double invnormv=1.0/normv;

 double refper=cx*(ny-cy)-cy*(nx-cx);//reference value for perpendicular component
 double difper=radius*invnormv*((ny-cy)*(ny-cy)-(-(nx-cx))*(nx-cx));
 double sttpll=cx*(nx-cx)+cy*(ny-cy);//parallel component value for segment start
 double endpll=nx*(nx-cx)+ny*(ny-cy);//parallel component value for segment end

 double pper=px*(ny-cy)-py*(nx-cx);
 double ppll=px*(nx-cx)+py*(ny-cy);
 if((std::abs(pper-refper)<difper)&&((sttpll-ppll)*(endpll-ppll)<=0)){
  result=true;
 } else {
  result=false;
 };
 //start circle
 //TODO We need modification about condition.
 //We should assert if p < c < n.
 if(result==false){
  double dist_from_start=sqrt((px-cx)*(px-cx)+(py-cy)*(py-cy));
  if(dist_from_start<sradius)result=true;
 };
 //end circle
 if(result==false){
  double dist_from_end=sqrt((px-nx)*(px-nx)+(py-ny)*(py-ny));
  if(dist_from_end<eradius)result=true;
 };
 return result;
}

bool PipelinePlanner::knowRobotPosition(const geometry_msgs::Pose pose,
int& ID,double& distance){
 ID=-1; distance=0.0;
 int npoints=check_points_.size();
 int lastpoint=(openclose_==1)?npoints:npoints-1;
 std::vector<double> radius,sradius,eradius;
 if(openclose_==1){
  radius.resize(npoints);
  sradius.resize(npoints);
  eradius.resize(npoints);
 } else {
  radius.resize(npoints-1);
  sradius.resize(npoints-1);
  eradius.resize(npoints-1);
 };
 //if(fix_pipe_radius_){
 // for(int i=0;i<sradius.size();i++){
 //  radius.at(i)=pipe_radius_;
 //  sradius.at(i)=pipe_radius_;
 //  eradius.at(i)=pipe_radius_;
 // };
 //} else {
  for(int i=0;i<sradius.size();i++){
   int ci=i;
   int pi=(ci==0)?((openclose_==2)?0:npoints-1):ci-1;
   int ni=(ci==npoints-1)?((openclose_==2)?ci:0):ci+1;
   //double cr=check_points_.at(ci).pose.position.z;
   //double pr=check_points_.at(pi).pose.position.z;
   //double nr=check_points_.at(ni).pose.position.z;
   double cr=TakeSegmentRadius(check_points_.at(ci).pose.position.z);
   double pr=TakeSegmentRadius(check_points_.at(pi).pose.position.z);
   double nr=TakeSegmentRadius(check_points_.at(ni).pose.position.z);
   radius.at(i)=cr;
   sradius.at(i)=std::min(cr,pr);
   eradius.at(i)=std::min(cr,nr);
  };
 //};
 std::vector<int> insegments,nearsegments;
 std::vector<asegment> segments;
 insegments.clear(); nearsegments.clear(); segments.clear();
 for(int i=0;i<lastpoint;i++){
  int part;double dist;
  bool inasegment=whereInaPipeSegment(pose,i,radius.at(i),sradius.at(i),eradius.at(i),part,dist,tolerance_);
  asegment thesegment;
  thesegment.id=i; thesegment.part=part; thesegment.dist=dist;
  thesegment.in=inasegment;
  segments.push_back(thesegment);
  if(inasegment){
   nearsegments.push_back(i);
   if(part==1)insegments.push_back(i);
  };
 };
 //to know real id to take
 int nnseg=nearsegments.size();
 int mostfrontindex;
 if(nearsegments.empty()){
  //error task
  return false;
 } else if(nnseg==check_points_.size()) {
  ROS_ERROR("All pipe segments include the robot position. Its strange.");
  return false;
 } else {
  //recognition of the queue
  int currentindex=nearsegments.at(0);
  int nextindex;
  for(int i=0;i<nnseg;i++){
   bool isNext=false;
   nextindex=(currentindex==0)?check_points_.size()-1:currentindex-1;
   for(int j=0;j<nnseg;j++){
    if(nextindex==nearsegments.at(j)){
     isNext=true; break;
    };
   };
   if(isNext){
    currentindex=nextindex;
   } else {
    break;
   };
  };
  mostfrontindex=currentindex;
  if(!insegments.empty()){
   int niseg=insegments.size();
   currentindex=mostfrontindex;
   for(int i=0;i<check_points_.size();i++){
    if(segments.at(currentindex).part==1){
     break;
    } else {
     currentindex=(currentindex+1)%(check_points_.size());
    };
   };
   mostfrontindex=currentindex;
  };
 };
 ID=mostfrontindex;
 if(segments.at(ID).part==1){
  distance=segments.at(mostfrontindex).dist;
 } else if(segments.at(ID).part==2){
  distance=-segments.at(mostfrontindex).dist;
 } else if(segments.at(ID).part==3){
  //distance=segments.at(mostfrontindex).dist;
  int ci=ID;
  int ni=(ci==check_points_.size()-1)?0:ci+1;
  double cx=check_points_.at(ci).pose.position.x;
  double cy=check_points_.at(ci).pose.position.y;
  double nx=check_points_.at(ni).pose.position.x;
  double ny=check_points_.at(ni).pose.position.y;
  distance=sqrt(pow(nx-cx,2.0)+pow(ny-cy,2.0))+segments.at(mostfrontindex).dist;
 };

 return true;
}

bool PipelinePlanner::whereInaPipeSegment(const geometry_msgs::Pose pose,
const int id,const double radius,const double sradius,const double eradius,
int& part,double &dist,double tolerance){
 bool result=false;
 part=0;
 int ci=id;
 int ni=(id==check_points_.size()-1)?0:ci+1;
 double cx=check_points_.at(ci).pose.position.x;
 double cy=check_points_.at(ci).pose.position.y;
 double nx=check_points_.at(ni).pose.position.x;
 double ny=check_points_.at(ni).pose.position.y;
 double px=pose.position.x;
 double py=pose.position.y;
 double normv=sqrt((nx-cx)*(nx-cx)+(ny-cy)*(ny-cy));
 if(normv<tolerance){
  ROS_ERROR_STREAM("The length of pipeline segment is too small."
                   " length: " << normv);
  return false;
 };
 double invnormv=1.0/normv;

 double refper=cx*(ny-cy)-cy*(nx-cx);//reference value for perpendicular component
 double difper=radius*invnormv*((ny-cy)*(ny-cy)-(-(nx-cx))*(nx-cx));
 double sttpll=cx*(nx-cx)+cy*(ny-cy);//parallel component value for segment start
 double endpll=nx*(nx-cx)+ny*(ny-cy);//parallel component value for segment end

 double pper=px*(ny-cy)-py*(nx-cx);
 double ppll=px*(nx-cx)+py*(ny-cy);
 if((std::abs(pper-refper)<difper)&&((sttpll-ppll)*(endpll-ppll)<=0)){
  result=true;
  part=1;
  dist=(ppll-sttpll)*invnormv;
  return result;
 } else {
  result=false;
 };
 //start circle
 if((sttpll-ppll)*(endpll-sttpll)>=0){
  double dist_from_start=sqrt((px-cx)*(px-cx)+(py-cy)*(py-cy));
  if(dist_from_start<sradius){
   result=true;
   part=2;
   dist=dist_from_start;
   return result;
  };
 };
 //end circle
 if((endpll-sttpll)*(ppll-endpll)>=0){
  double dist_from_end=sqrt((px-nx)*(px-nx)+(py-ny)*(py-ny));
  if(dist_from_end<eradius){
   result=true;
   part=3;
   dist=dist_from_end;
   return result;
  };
 };
 return false;

}

bool PipelinePlanner::isBehindInaSegment(int idx,
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal){
 int ci,ni;
 double cx,cy,nx,ny; double sx,sy,gx,gy;
 double poss,posg;
 bool result;

 ci=idx; ni=(ci==check_points_.size()-1)?0:ci+1;
 cx=check_points_.at(ci).pose.position.x;
 cy=check_points_.at(ci).pose.position.y;
 nx=check_points_.at(ni).pose.position.x;
 ny=check_points_.at(ni).pose.position.y;
 sx=start.pose.position.x;
 sy=start.pose.position.y;
 gx= goal.pose.position.x;
 gy= goal.pose.position.y;

 poss=(nx-cx)*sx+(ny-cy)*sy;
 posg=(nx-cx)*gx+(ny-cy)*gy;
 if(poss>posg){
  result=true;
 } else {
  result=false;
 };
 return result;
};

bool PipelinePlanner::VeilCostmap(const std::vector<int> segids,
                 const int startindex,const int goalindex,const geometry_msgs::PoseStamped& start){
 unsigned int costsizex=costmap_->getSizeInCellsX();
 unsigned int costsizey=costmap_->getSizeInCellsY();
 int numsegids=segids.size();
 std::vector<int> daub;//bool
 daub.clear();
 daub.insert(daub.begin(),costsizex*costsizey,1);
 double map_resolution=costmap_->getResolution();
 //torch model
 double origin_x=costmap_->getOriginX();
 double origin_y=costmap_->getOriginY();
 const double robot_x=start.pose.position.x;
 const double robot_y=start.pose.position.y;
 int min_visible_x,min_visible_y,max_visible_x,max_visible_y;
 int min_index_x,max_index_x,min_index_y,max_index_y;
 if(use_torch_){
  min_visible_x=(int)((robot_x-torch_area_x_-origin_x)/map_resolution);
  max_visible_x=(int)((robot_x+torch_area_x_-origin_x)/map_resolution);
  min_visible_x=std::max(min_visible_x,0);
  max_visible_x=std::min(max_visible_x,(int)costsizex);
  min_visible_y=(int)((robot_y-torch_area_y_-origin_y)/map_resolution);
  max_visible_y=(int)((robot_y+torch_area_y_-origin_y)/map_resolution);
  min_visible_y=std::max(min_visible_y,0);
  max_visible_y=std::min(max_visible_y,(int)costsizey);
  min_index_x=min_visible_x; max_index_x=max_visible_x;
  min_index_y=min_visible_y; max_index_y=max_visible_y;
 } else {
  min_index_x=0; max_index_x=costsizex;
  min_index_y=0; max_index_y=costsizey;
 };
 //veil loop
 for(int i=0;i<numsegids;i++){
  double radius,sradius,eradius;
  int ci=segids.at(i);
  int ni=(ci==check_points_.size()-1)?0:ci+1;
  int pi;
  //if(fix_pipe_radius_){
  // radius=pipe_radius_;
  // sradius=pipe_radius_;
  // eradius=pipe_radius_;
  //} else {
   pi=(ci==0)?check_points_.size()-1:ci-1;
   double cr=TakeSegmentRadius(check_points_.at(ci).pose.position.z);
   double pr=TakeSegmentRadius(check_points_.at(pi).pose.position.z);
   double nr=TakeSegmentRadius(check_points_.at(ni).pose.position.z);
   //radius=check_points_.at(ci).pose.position.z;
   radius=cr;
   if(openclose_==2&&ci==0){
    sradius=radius;
   } else {
    //sradius=std::min(radius,check_points_.at(pi).pose.position.z);
    sradius=std::min(radius,pr);
   };
   if(openclose_==2&&ci==check_points_.size()-2){
    eradius=radius;
   } else {
    //eradius=std::min(radius,check_points_.at(ni).pose.position.z);
    eradius=std::min(radius,nr);
   };
  //};
  double cx=check_points_.at(ci).pose.position.x;
  double cy=check_points_.at(ci).pose.position.y;
  double nx=check_points_.at(ni).pose.position.x;
  double ny=check_points_.at(ni).pose.position.y;

  double normv=std::sqrt((nx-cx)*(nx-cx)+(ny-cy)*(ny-cy));
  double invnormv=1.0/normv;
  double refper=cx*(ny-cy)-cy*(nx-cx); //reference value for perpendicular component
  double difper=radius*invnormv*((ny-cy)*(ny-cy)-(-(nx-cx))*(nx-cx))
               +map_resolution*(0.5* std::abs(ny-cy)+0.5* std::abs(nx-cx));
  double refpll=(0.5*(nx+cx))*(nx-cx)+(0.5*(ny+cy))*(ny-cy);//reference value for parallel component
  //reference value for parallel component
  double difpll=(0.5*(nx-cx))*(nx-cx)+(0.5*(ny-cy))*(ny-cy)
               +map_resolution*(0.5*std::abs(nx-cx)+0.5*std::abs(ny-cy));
  double sedgepll,eedgepll,startpll,respll;
  int start_area=0;
  if(ci==startindex){
   sedgepll=(cx)*(nx-cx)+(cy)*(ny-cy);//parallel component for the start edge
   eedgepll=(nx)*(nx-cx)+(ny)*(ny-cy);//parallel component for the end edge
   startpll=(start.pose.position.x)*(nx-cx)+(start.pose.position.y)*(ny-cy);//parallel component for the start point
   //respll=map_resolution*(0.5*std::abs(nx-cx)+0.5*std::abs(ny-cy));
   if((sedgepll-startpll)*(eedgepll-sedgepll)>0){
    start_area=1;//start point is out from start edge
   } else if((startpll-sedgepll)*(eedgepll-startpll)>0){
    start_area=2;//start point is in the first pipe segment
   } else {
    start_area=3;//start point is out from end edge
   };
  };
  //for(unsigned int xi=0;xi<costsizex;xi++)
  //for(unsigned int yi=0;yi<costsizey;yi++)
  for(unsigned int xi=min_index_x;xi<max_index_x;xi++){
  for(unsigned int yi=min_index_y;yi<max_index_y;yi++){
   int index=yi*costsizex+xi;
   if(daub.at(index)==2)continue;
   if(pipeline_.data[index]==100){
    daub.at(index)=2;continue;
   };
   double wx,wy;
   costmap_->mapToWorld(xi,yi,wx,wy);
   //pipeline rectangular
   double curper=wx*(ny-cy)-wy*(nx-cx);
   double curpll=wx*(nx-cx)+wy*(ny-cy);
   if((std::abs(curper-refper) < difper)&&(std::abs(curpll-refpll) < difpll)){
    if((ci==startindex)&&((start_area==2
      &&((curpll-sedgepll)*(startpll-curpll)>0)
      &&(std::abs(curpll-startpll)>0.1))
      ||(start_area==3))){
     //plug of start pipesegment
     //daub.at(costmap_->getIndex(xi,yi))=1;
     daub.at(index)=2;
     continue;
    } else {
     //daub.at(costmap_->getIndex(xi,yi))=0;
     daub.at(index)=0;
    };
   };
   //pipelince connection circle
   double distancefromconnection; double startfromconnection;
   //start edge circle
   distancefromconnection=sqrt((wx-cx)*(wx-cx)+(wy-cy)*(wy-cy));
   if(distancefromconnection<sradius+map_resolution*(0.5*sqrt(2.0))){
    //we should set condition to plug start circle
    startfromconnection=sqrt((start.pose.position.x-cx)*(start.pose.position.x-cx)+
        (start.pose.position.y-cy)*(start.pose.position.y-cy));
    if((ci==startindex)&&start_area==1
       &&((sedgepll-curpll)*(eedgepll-sedgepll)>0)
       &&(distancefromconnection>(startfromconnection+sradius)*0.5)){
     //daub.at(costmap_->getIndex(xi,yi))=2;
     daub.at(index)=2;
     continue;
    } else {
     daub.at(index)=0;
    };
   };
   //end edge circle
   distancefromconnection=sqrt((wx-nx)*(wx-nx)+(wy-ny)*(wy-ny));
   if(distancefromconnection<eradius+map_resolution*(0.5*sqrt(2.0))){
    daub.at(index)=0;
   };
  };
  };
 };

 //preservation of old costmap and replacement of costmap
 for(unsigned int xi=0;xi<costsizex;xi++){
 for(unsigned int yi=0;yi<costsizey;yi++){
  unsigned int idx=costmap_->getIndex(xi,yi);
  unsigned char curr_cost=costmap_->getCost(xi,yi);
  bool notVisible=(xi<min_index_x)||(max_index_x<=xi)||(yi<min_index_y)||(max_index_y<=yi);
  if(notVisible){
   navfn_costmap_->setCost(xi,yi,curr_cost);
  } else if((daub.at(idx)==1)||(daub.at(idx)==2)){
   navfn_costmap_->setCost(xi,yi,lethal_cost_);
  } else {
   unsigned char new_cost;
   if((curr_cost!=253)&&(curr_cost!=254)&&(curr_cost!=255)){
    //new_cost=(unsigned char)(curr_cost
    //           +centre_weight_*dist_from_pipe_centre_.at(idx));
    new_cost=std::min(lethal_cost_,(unsigned char)(curr_cost
               +centre_weight_*dist_from_pipe_centre_.at(idx)));
   } else {
    new_cost=curr_cost;
   };
   navfn_costmap_->setCost(xi,yi,new_cost);
  };
 };};

 return true;
}

#if MODE==MODE_GPU
bool PipelinePlanner::VeilCostmap_cuda(const std::vector<int> segids,
                 const int startindex,const int goalindex,const geometry_msgs::PoseStamped& start){
 ThreadNumberAdjusment(false);

 //preparations
 cudaError_t error_allocation;
 //stones
 int numstones=segids.size();
 double *stonex,*stoney;
 //stonex=new double[numstones+1];
 //stoney=new double[numstones+1];
 if(debug){
  error_allocation=cudaMallocManaged(&stonex,(numstones+1)*sizeof(double));
  ROS_INFO_STREAM("error stonex: " << error_allocation);//for debug
  error_allocation=cudaMallocManaged(&stoney,(numstones+1)*sizeof(double));
  ROS_INFO_STREAM("error stoney: " << error_allocation);//for debug
 } else {
  cudaMallocManaged(&stonex,(numstones+1)*sizeof(double));
  cudaMallocManaged(&stoney,(numstones+1)*sizeof(double));
 };
 for(int i=0;i<numstones;i++){
  int index=segids.at(i);
  stonex[i]=check_points_.at(index).pose.position.x;
  stoney[i]=check_points_.at(index).pose.position.y;
  if(i==numstones-1){
   int lastindex=(index==check_points_.size()-1)?0:index+1;
   stonex[i+1]=check_points_.at(lastindex).pose.position.x;
   stoney[i+1]=check_points_.at(lastindex).pose.position.y;
  };
 };
 double *radius, *sradius, *eradius;
 if(debug){
  error_allocation=cudaMallocManaged(&radius,(numstones)*sizeof(double));
  ROS_INFO_STREAM("error radius: " << error_allocation);//for debug
  error_allocation=cudaMallocManaged(&sradius,(numstones)*sizeof(double));
  ROS_INFO_STREAM("error sradius: " << error_allocation);//for debug
  error_allocation=cudaMallocManaged(&eradius,(numstones)*sizeof(double));
  ROS_INFO_STREAM("error eradius: " << error_allocation);//for debug
 } else {
  cudaMallocManaged(&radius,(numstones)*sizeof(double));
  cudaMallocManaged(&sradius,(numstones)*sizeof(double));
  cudaMallocManaged(&eradius,(numstones)*sizeof(double));
 };
 //if(fix_pipe_radius_){
 // for(int i=0;i<numstones;i++){
 //  radius[i]=pipe_radius_;
 //  sradius[i]=pipe_radius_;
 //  eradius[i]=pipe_radius_;
 // };
 //} else {
  for(int i=0;i<numstones;i++){
   int ci=segids.at(i);
   int ni=(ci==check_points_.size()-1)?0:ci+1;
   int pi=(ci==0)?check_points_.size()-1:ci-1;
   double cr=TakeSegmentRadius(check_points_.at(ci).pose.position.z);
   double pr=TakeSegmentRadius(check_points_.at(pi).pose.position.z);
   double nr=TakeSegmentRadius(check_points_.at(ni).pose.position.z);
   //radius[i]=check_points_.at(ci).pose.position.z;
   //sradius[i]=(openclose_==2&&ci==0)?
   //  radius[i]:std::min(radius[i],check_points_.at(pi).pose.position.z);
   //eradius[i]=(openclose_==2&&ci==check_points_.size()-2)?
   //  radius[i]:std::min(radius[i],check_points_.at(ni).pose.position.z);
   radius[i]=cr;
   sradius[i]=(openclose_==2&&ci==0)?
     radius[i]:std::min(radius[i],pr);
   eradius[i]=(openclose_==2&&ci==check_points_.size()-2)?
     radius[i]:std::min(radius[i],nr);
  };
 //};
 //start position
 double startx=start.pose.position.x;
 double starty=start.pose.position.y;
 //radius
 //double radius=pipe_radius_;
 //size and resolution of costmap
 unsigned int costsizex=costmap_->getSizeInCellsX();
 unsigned int costsizey=costmap_->getSizeInCellsY();
 double map_resolution=costmap_->getResolution();
 //origin
 double origin_x=costmap_->getOriginX();
 double origin_y=costmap_->getOriginY();

 //torch model
 int min_visible_x,min_visible_y,max_visible_x,max_visible_y;
 int min_index_x,max_index_x,min_index_y,max_index_y;
 if(use_torch_){
  min_visible_x=(int)((startx-torch_area_x_-origin_x)/map_resolution);
  max_visible_x=(int)((startx+torch_area_x_-origin_x)/map_resolution);
  min_visible_x=std::max(min_visible_x,0);
  max_visible_x=std::min(max_visible_x,(int)costsizex);
  min_visible_y=(int)((starty-torch_area_y_-origin_y)/map_resolution);
  max_visible_y=(int)((starty+torch_area_y_-origin_y)/map_resolution);
  min_visible_y=std::max(min_visible_y,0);
  max_visible_y=std::min(max_visible_y,(int)costsizey);
  min_index_x=min_visible_x; max_index_x=max_visible_x;
  min_index_y=min_visible_y; max_index_y=max_visible_y;
 } else {
  min_index_x=0; max_index_x=costsizex;
  min_index_y=0; max_index_y=costsizey;
 };

 //new cost array
 unsigned char *newCost;
 if(debug){
  error_allocation=cudaMallocManaged(&newCost,(costsizex*costsizey)*sizeof(unsigned char));
  ROS_INFO_STREAM("error newCost: " << error_allocation);//for debug
  ROS_INFO_STREAM("07 costsizex*costsizey: " << costsizex*costsizey);//for debug
 } else {
  cudaMallocManaged(&newCost,(costsizex*costsizey)*sizeof(unsigned char));
 };
 for(int xi=0;xi<costsizex;xi++){
 for(int yi=0;yi<costsizey;yi++){
  int index=yi*costsizex+xi;
  //newCost[index]=costmap_->getCost(xi,yi);
  unsigned char c=costmap_->getCost(xi,yi);
  newCost[index]=c;
 };};
 //dist_from_pipe_centre
 double *dist_from_centre;
 if(debug){
  error_allocation=cudaMallocManaged(&dist_from_centre,(costsizex*costsizey)*sizeof(double));
  ROS_INFO_STREAM("error dist from centre: " << error_allocation);//for debug
 } else {
  cudaMallocManaged(&dist_from_centre,(costsizex*costsizey)*sizeof(double));
 };
 //for(int xi=0;xi<costsizex;xi++)
 //for(int yi=0;yi<costsizey;yi++)
 for(int xi=min_index_x;xi<max_index_x;xi++){
 for(int yi=min_index_y;yi<max_index_y;yi++){
  int index=yi*costsizex+xi;
  dist_from_centre[index]=dist_from_pipe_centre_.at(index);
 };};

 //call of cuda function
 //using num_threads_
 calling_device(numstones,stonex,stoney,startx,starty,radius,sradius,eradius,
 costsizex,costsizey,map_resolution,origin_x,origin_y,dist_from_centre,
 centre_weight_,lethal_cost_,min_index_x,max_index_x,min_index_y,max_index_y,
 newCost,num_threads_,debug);

 //replacement of costmap
 for(int xi=0;xi<costsizex;xi++){
 for(int yi=0;yi<costsizey;yi++){
  //int idx=costmap_->getIndex(xi,yi);
  int idx=yi*costsizex+xi;
  //costmap_->setCost(xi,yi,newCost[idx]);
  navfn_costmap_->setCost(xi,yi,newCost[idx]);
 };};

 //free allocated variables
 cudaError_t error_free;
 if(debug){
  error_free=cudaFree(stonex);
  ROS_INFO_STREAM("free stonex: " << error_free);
  error_free=cudaFree(stoney);
  ROS_INFO_STREAM("free stoney: " << error_free);
  error_free=cudaFree(newCost);
  ROS_INFO_STREAM("free newCost: " << error_free);
  error_free=cudaFree(dist_from_centre);
  ROS_INFO_STREAM("free dist_from_centre: " << error_free);
 } else {
  cudaFree(stonex);
  cudaFree(stoney);
  cudaFree(newCost);
  cudaFree(dist_from_centre);
 };

cudaError_t last_error;
last_error=cudaGetLastError();
if(last_error!=cudaSuccess){
 ROS_ERROR_STREAM("cudaGetLastError : " << last_error);
} else if(debug){
 ROS_INFO_STREAM("cudaGetLastError : " << last_error);
};

 return true;
}
#endif

bool PipelinePlanner::ConnectPoints(const geometry_msgs::PoseStamped& start,
         const geometry_msgs::PoseStamped& goal, const std::vector<int> segids,
         std::vector<geometry_msgs::PoseStamped>& plan){
 geometry_msgs::PoseStamped astone;
 ros::Time plan_time;
 //if(!fix_pipe_radius_){
  plan_time = ros::Time::now();
  astone.header.stamp = plan_time;
  astone.header.frame_id = gained_frame_id_;
  astone.pose.position.x=0.0;//x
  astone.pose.position.y=0.0;//y
  astone.pose.position.z=0.0;
  astone.pose.orientation.x=0.0;
  astone.pose.orientation.y=0.0;
  astone.pose.orientation.z=0.0;
  astone.pose.orientation.w=1.0;
 //};
 plan.clear();
 if(segids.size()==1){
  plan.push_back(start);
  if(!(DrawOpenSegment(start, goal, plan))){return false;};
  plan.push_back(goal);
  return true;
 };
 plan.push_back(start);
 if(!(DrawOpenSegment(start, check_points_.at(segids.at(1)), plan))){return false;};
 for(int i=1;i<segids.size()-1;i++){
  //if(fix_pipe_radius_){
  // astone=check_points_.at(segids.at(i));
  //} else {
   astone.pose.position.x=check_points_.at(segids.at(i)).pose.position.x;
   astone.pose.position.y=check_points_.at(segids.at(i)).pose.position.y;
  //};
  plan.push_back(astone);
  if(!(DrawOpenSegment(check_points_.at(segids.at(i)),
                check_points_.at(segids.at(i+1)), plan))){return false;};
 };
 //if(fix_pipe_radius_){
 // astone=check_points_.at(segids.at(segids.size()-1));
 //} else {
  astone.pose.position.x=check_points_.at(segids.at(segids.size()-1)).pose.position.x;
  astone.pose.position.y=check_points_.at(segids.at(segids.size()-1)).pose.position.y;
 //};
 plan.push_back(astone);
 if(!(DrawOpenSegment(check_points_.at(segids.at(segids.size()-1)),
                goal, plan))){return false;};
 plan.push_back(goal);

 return true;
}

bool PipelinePlanner::DrawOpenSegment(const geometry_msgs::PoseStamped init_point,
                const geometry_msgs::PoseStamped end_point,
                std::vector<geometry_msgs::PoseStamped>& plan){
 double map_resolution=costmap_->getResolution();
 double l=map_resolution*2.0;
 double ix=init_point.pose.position.x;
 double iy=init_point.pose.position.y;
 double ex=end_point.pose.position.x;
 double ey=end_point.pose.position.y;

 double diff_ie=sqrt((ex-ix)*(ex-ix)+(ey-iy)*(ey-iy));
 int num=(int) (diff_ie/l);
 if(num==0)return true;
 double dx=(ex-ix)/(num+1.0);
 double dy=(ey-iy)/(num+1.0);

 ros::Time plan_time = ros::Time::now();

 for(int i=1;i<num+1;i++){
  geometry_msgs::PoseStamped astone;
  double x=ix+dx*i;
  double y=iy+dy*i;
  unsigned int px,py;
  if(!(costmap_->worldToMap(x,y,px,py))){
   ROS_ERROR_STREAM("out of map: (" << x << "," << y << ")");
   return false;
  };
  if(!charge_){
   unsigned char curr_cost=costmap_->getCost(px,py);
   if(curr_cost >= lethal_cost_){
    ROS_ERROR_STREAM("point (" << x << "," << y << ") is lethal");
    return false;
   };
  };

  astone.header.stamp = plan_time;
  astone.header.frame_id = gained_frame_id_;
  astone.pose.position.x=x;
  astone.pose.position.y=y;
  astone.pose.position.z=0.0;
  astone.pose.orientation.x=0.0;
  astone.pose.orientation.y=0.0;
  astone.pose.orientation.z=0.0;
  astone.pose.orientation.w=1.0;
  plan.push_back(astone);
 };
 return true;
}

void PipelinePlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
 nav_msgs::Path path_publish;
 path_publish.poses.resize(path.size());

 if(!path.empty()){
  path_publish.header.frame_id = path[0].header.frame_id;
  path_publish.header.stamp = path[0].header.stamp;
 };
 for(int i=0;i<path.size();i++){
  path_publish.poses[i]=path[i];
 };
 pub_plan_.publish(path_publish);

}

void PipelinePlanner::makePipeline(){
 makePipeline(true);
}


void PipelinePlanner::makePipeline(bool legal){
 const int costsizex=costmap_->getSizeInCellsX();
 const int costsizey=costmap_->getSizeInCellsY();
 const double map_resolution=costmap_->getResolution();
 const double origin_x=costmap_->getOriginX();
 const double origin_y=costmap_->getOriginY();

 //nav_msgs::OccupancyGrid pipeline_;
 pipeline_.header.stamp=ros::Time::now();
 pipeline_.header.frame_id=gained_frame_id_;
 pipeline_.info.resolution=map_resolution;
 pipeline_.info.width=costsizex;
 pipeline_.info.height=costsizey;
 pipeline_.info.origin.position.x=origin_x;
 pipeline_.info.origin.position.y=origin_y;
 pipeline_.info.origin.position.z=0.0;
 pipeline_.info.origin.orientation.w=1.0;
 pipeline_.data.resize(pipeline_.info.width*pipeline_.info.height);

 if(legal==false){
  for(int i=0;i<costsizex*costsizey;i++){
   pipeline_.data[i]=100;
  };
  pub_pipeline_.publish(pipeline_);
  return;
 };

 int numchecks=check_points_.size();
 int numsegids=(openclose_==1)?numchecks:numchecks-1;

 if(numchecks<2){
  for(int i=0;i<costsizex*costsizey;i++){
   pipeline_.data[i]=100;
  };
  pub_pipeline_.publish(pipeline_);
  return;
 };

 double *stonex,*stoney,*radii,*sradii,*eradii;
 int *data;
 bool isClose;
 isClose=(openclose_==1)?true:false;
 //GPU mode
 //cudaMallocManaged(&stonex,(numchecks)*sizeof(double));
 //cudaMallocManaged(&stoney,(numchecks)*sizeof(double));
 //cudaMallocManaged(&radii,(numsegids)*sizeof(double));
 //cudaMallocManaged(&sradii,(numsegids)*sizeof(double));
 //cudaMallocManaged(&eradii,(numsegids)*sizeof(double));
 //cudaMallocManaged(&data,(costsizex*costsizey)*sizeof(int));
 //CPU mode
 stonex=new double[numchecks];
 stoney=new double[numchecks];
 radii=new double[numsegids];
 sradii=new double[numsegids];
 eradii=new double[numsegids];
 data=new int[costsizex*costsizey];

 for(int i=0;i<numchecks;i++){
  stonex[i]=check_points_.at(i).pose.position.x;
  stoney[i]=check_points_.at(i).pose.position.y;
 };
 //if(fix_pipe_radius_){
 // for(int i=0;i<numsegids;i++){
 //  radii[i]=pipe_radius_;sradii[i]=pipe_radius_;eradii[i]=pipe_radius_;
 // };
 //} else {
  for(int i=0;i<numsegids;i++){
   int ci=i;
   int pi=(ci==0)?numchecks-1:ci-1;
   int ni=(ci==numchecks-1)?0:ci+1;
   double cr=TakeSegmentRadius(check_points_.at(ci).pose.position.z);
   double pr=TakeSegmentRadius(check_points_.at(pi).pose.position.z);
   double nr=TakeSegmentRadius(check_points_.at(ni).pose.position.z);
   //radii[i]=check_points_.at(i).pose.position.z;
   radii[ci]=cr;
   if(openclose_==2&&ci==0){
    sradii[ci]=radii[ci];
   } else {
    //sradii[ci]=std::min(radii[ci],check_points_.at(pi).pose.position.z);
    sradii[ci]=std::min(radii[ci],pr);
   };
   if(openclose_==2&&ci==numchecks-2){
    eradii[ci]=radii[ci];
   } else {
    //eradii[ci]=std::min(radii[ci],check_points_.at(ni).pose.position.z);
    eradii[ci]=std::min(radii[ci],nr);
   };
  };
 //};
 //GPU
// calcPipeline_device(numchecks,stonex,stoney,isClose,radii,sradii,eradii,
//   origin_x,origin_y,costsizex,costsizey,map_resolution,data,num_threads_);
 //CPU
 calcPipeline(numchecks,stonex,stoney,isClose,radii,sradii,eradii,
   origin_x,origin_y,costsizex,costsizey,map_resolution,data);
 for(int i=0;i<costsizex*costsizey;i++){
  pipeline_.data[i]=data[i];
 };
 //termination
 //GPU
 //cudaFree(stonex);
 //cudaFree(stoney);
 //cudaFree(radii);
 //cudaFree(sradii);
 //cudaFree(eradii);
 //cudaFree(data);
 //CPU
 delete[] stonex;
 delete[] stoney;
 delete[] radii;
 delete[] sradii;
 delete[] eradii;
 delete[] data;

 pub_pipeline_.publish(pipeline_);
}

void PipelinePlanner::calcPipeline(const int numcheckpoints,
const double *stonex,const double *stoney,const bool isClose,
const double *radii,const double *sradii,const double *eradii,
const double origin_x,const double origin_y,
const unsigned int costsizex,const unsigned int costsizey,
const double resolution,int *data){
 //initialisation
 for(int i=0;i<costsizex*costsizey;i++){
  data[i]=100;
 };
 int numsegids=(isClose==true)?numcheckpoints:numcheckpoints-1;
 //calculation
 for(int i=0;i<numsegids;i++){
  int ci=i;
  int ni=(ci==numcheckpoints-1)?0:ci+1;
  const double cx=stonex[ci];
  const double cy=stoney[ci];
  const double nx=stonex[ni];
  const double ny=stoney[ni];
  const double normv=std::sqrt((nx-cx)*(nx-cx)+(ny-cy)*(ny-cy));
  const double invnormv=1.0/normv;
  const double refper=cx*(ny-cy)-cy*(nx-cx); //reference value for perpendicular component
  const double difper=radii[i]*invnormv*((ny-cy)*(ny-cy)-(-(nx-cx))*(nx-cx))
               +resolution*(0.5* std::abs(ny-cy)+0.5* std::abs(nx-cx));
  const double refpll=(0.5*(nx+cx))*(nx-cx)+(0.5*(ny+cy))*(ny-cy);//reference value for parallel component
  const double difpll=(0.5*(nx-cx))*(nx-cx)+(0.5*(ny-cy))*(ny-cy)
               +resolution*(0.5*std::abs(nx-cx)+0.5*std::abs(ny-cy));

  //restrict region for fast calculation
  int min_index_x,min_index_y,max_index_x,max_index_y;
  double min_x,min_y,max_x,max_y;

  min_x=std::min(std::min(cx-radii[i],cx-sradii[i]),std::min(nx-radii[i],nx-eradii[i]));
  min_index_x=std::max(0,(int)((min_x-origin_x)/resolution));
  max_x=std::max(std::max(cx+radii[i],cx+sradii[i]),std::max(nx+radii[i],nx+eradii[i]));
  max_index_x=std::min((int)costsizex,(int)((max_x-origin_x)/resolution));

  min_y=std::min(std::min(cy-radii[i],cy-sradii[i]),std::min(ny-radii[i],ny-eradii[i]));
  min_index_y=std::max(0,(int)((min_y-origin_y)/resolution));
  max_y=std::max(std::max(cy+radii[i],cy+sradii[i]),std::max(ny+radii[i],ny+eradii[i]));
  max_index_y=std::min((int)costsizey,(int)((max_y-origin_y)/resolution));

  //ROS_INFO_STREAM("x: " << min_index_x << "," << max_index_x << ", y: " << min_index_y << "," << max_index_y);
  //min_index_x=0;min_index_y=0;
  //max_index_x=costsizex;max_index_y=costsizey;

  //for(int xi=0;xi<costsizex;xi++)
  //for(int yi=0;yi<costsizey;yi++)
  for(int xi=min_index_x;xi<max_index_x;xi++){
  for(int yi=min_index_y;yi<max_index_y;yi++){
   int index=yi*costsizex+xi;
   if(data[index]==0)continue;
   const double wx=origin_x+(xi+0.5)*resolution;
   const double wy=origin_y+(yi+0.5)*resolution;
   const double curper=wx*(ny-cy)-wy*(nx-cx);
   const double curpll=wx*(nx-cx)+wy*(ny-cy);
   //rectangular region
   if((std::abs(curper-refper)<difper)&&(std::abs(curpll-refpll)<difpll)){
    data[index]=0;
    continue;
   } else {
    //start circle region
    const double distfromstart=sqrt(pow(wx-cx,2.0)+pow(wy-cy,2.0));
    if(distfromstart<sradii[i]+resolution*0.5*sqrt(2.0)){
     data[index]=0;
     continue;
    };
    //end circle region
    const double distfromend=sqrt(pow(wx-nx,2.0)+pow(wy-ny,2.0));
    if(distfromend<eradii[i]+resolution*0.5*sqrt(2.0)){
     data[index]=0;
    };
   };
  };};
 };
}

void PipelinePlanner::callbackgoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal){
 willDisplay_=true;
}

void PipelinePlanner::callbackRobotPose(const geometry_msgs::Pose::ConstPtr& pose){
 boost::mutex::scoped_lock lock(mutex_robot_position_);
 robot_position_.position.x=pose->position.x;
 robot_position_.position.y=pose->position.y;

}

void PipelinePlanner::informRobotPose(){
 ros::NodeHandle nh;
 ros::Rate r(1.0);
 ROS_INFO("Pipeline planner: informRobotPose loop started");
 while(ros::ok()){
  bool in; int id; double distance;
  pipeline_planner::RobotPosition position;
  if(read_status_.data==5||read_status_.data==3){
   in=knowRobotPosition(robot_position_,id,distance);
   position.in=in;
   position.id=id;
   position.startdist=distance;
   position.previd=-1;
   position.nextid=-1;
   position.enddist=0.0;
   if(in){
    for(std::vector<pipeline_planner::SegmentInfo>::iterator it=segment_infos_.begin(); it!=segment_infos_.end();it++){
     if(it->ID == id){
      position.previd=it->prevID;
      position.nextid=it->nextID;
      position.enddist= it->length - distance;
     };
    };
   };
  } else {
   position.in=false;
   position.id=-1;
   position.previd=-1;
   position.nextid=-1;
   position.startdist=0.0;
   position.enddist=0.0;
  };
  for(std::vector<pipeline_planner::SegmentInfo>::iterator it=segment_infos_.begin();it!=segment_infos_.end();it++){
   if(it->ID == position.id){it->start=true;}else{it->start=false;};
   if(it->nextID == position.id){it->end=true;}else{it->end=false;};
  };
  pub_robot_position_.publish(position);
  r.sleep();
 };

}

void PipelinePlanner::TimeDisplay(const bool time_display,const double duration){
 if(time_display){
  ROS_WARN_STREAM("/ **                         **  ");
  ROS_WARN_STREAM("        Pipeline planner         ");
  ROS_WARN_STREAM("      makePlan duration time     ");
  ROS_WARN_STREAM("        " << duration);
  ROS_WARN_STREAM("  **                         ** /");
 };
 consumed_time_.data=duration;
 pub_time_.publish(consumed_time_);
}

bool PipelinePlanner::TakeLocalGoal(const geometry_msgs::PoseStamped& start,
const geometry_msgs::PoseStamped& goal,geometry_msgs::PoseStamped& local_goal,
int& err){
 err=0;
 //to assert we use torch model
 if(!use_torch_){
  ROS_ERROR("we only use TakeLocalGoal function on torch model");
  err=1;
  return false;
 };
 //preparation
 bool in; int sid,gid; double sdistance,gdistance;
 geometry_msgs::Pose pose;
 int nextid;
 double sx,sy,gx,gy;//start and goal
 sx=start.pose.position.x; sy=start.pose.position.y;
 gx=goal.pose.position.x; gy=goal.pose.position.y;
 double cx,cy,nx,ny; double lenseg;
 double pseusx,pseusy,pseugx,pseugy;//pseudo-start and pseudo-goal
 double crossx,crossy;//cross point
 int result;
 int numid;
 int numcp=check_points_.size();
 //to determine pseudo-start
 pose.position.x=sx;
 pose.position.y=sy;
 in=knowRobotPosition(pose,sid,sdistance);
 if(!in){
  ROS_ERROR("start position is out of pipeline");
  err=2;
  return false;
 };
 //to determine pseudo-goal
 pose.position.x=gx;
 pose.position.y=gy;
 in=knowRobotPosition(pose,gid,gdistance);
 if(!in){
  ROS_ERROR("goal position is out of pipeline");
  err=3;
  return false;
 };
 //substitution of pseudo-start and pseudo-end values
 //nextid=(sid==check_points_.size()-1)?0:sid+1;
 nextid=(sid+1)%numcp;
 cx=check_points_.at(sid).pose.position.x;
 cy=check_points_.at(sid).pose.position.y;
 nx=check_points_.at(nextid).pose.position.x;
 ny=check_points_.at(nextid).pose.position.y;
 lenseg=sqrt(pow(nx-cx,2)+pow(ny-cy,2));
 if(lenseg<tolerance_){
  ROS_ERROR_STREAM("the length of pipe segment " << sid << " is too short: " << lenseg);
  err=4;
  return false;
 };
 pseusx=(nx-cx)*sdistance/lenseg+cx;
 pseusy=(ny-cy)*sdistance/lenseg+cy;
 nextid=(gid+1)%numcp;
 cx=check_points_.at(gid).pose.position.x;
 cy=check_points_.at(gid).pose.position.y;
 nx=check_points_.at(nextid).pose.position.x;
 ny=check_points_.at(nextid).pose.position.y;
 lenseg=sqrt(pow(nx-cx,2)+pow(ny-cy,2));
 if(lenseg<tolerance_){
  ROS_ERROR_STREAM("the length of pipe segment " << sid << " is too short: " << lenseg);
  err=4;
  return false;
 };
 pseugx=(nx-cx)*gdistance/lenseg+cx;
 pseugy=(ny-cy)*gdistance/lenseg+cy;
 //to check whether pipe segment crosses the torch area for each pipe segment
 if(sid<=gid){ numid=gid-sid;}else{numid=gid-sid+numcp;};
 for(int i=sid;i<sid+numid+1;i++){
  int ID=i%numcp; int nID=(i+1)%numcp;
  if(i==sid){
   cx=pseusx; cy=pseusy;
  } else {
   cx=check_points_.at(ID).pose.position.x;
   cy=check_points_.at(ID).pose.position.y;
  };
  if(i==sid+numid){
   nx=pseugx; ny=pseugy;
  } else {
   nx=check_points_.at(nID).pose.position.x;
   ny=check_points_.at(nID).pose.position.y;
  };
  result=CrossTorchArea(sx,sy,cx,cy,nx,ny,crossx,crossy);
  if(result<0){
   ROS_ERROR("unexpected error, torch area might be too narrow with respect to the pipeline radius");
  ROS_ERROR("please adjust torch_area_x and torch_area_y");
   err=5;
   return false;
  } else if(result==1){
   ROS_ERROR("pseudo-start position is out of torch area");
   err=6;
   return false;
  } else if(result==2){
   //substitution of local goal and return true
   local_goal.header.seq=goal.header.seq;
   local_goal.header.stamp=goal.header.stamp;
   local_goal.header.frame_id=goal.header.frame_id;
   local_goal.pose.position.z=goal.pose.position.z;
   local_goal.pose.orientation.x=goal.pose.orientation.x;
   local_goal.pose.orientation.y=goal.pose.orientation.y;
   local_goal.pose.orientation.z=goal.pose.orientation.z;
   local_goal.pose.orientation.w=goal.pose.orientation.w;
   //main difference
   local_goal.pose.position.x=crossx;
   local_goal.pose.position.y=crossy;
   return true;
  };
 };
 //case in that the path from pseudo-start to pseudo-goal is in the torch area
 local_goal.header.seq=goal.header.seq;
 local_goal.header.stamp=goal.header.stamp;
 local_goal.header.frame_id=goal.header.frame_id;
 local_goal.pose.position.z=goal.pose.position.z;
 local_goal.pose.orientation.x=goal.pose.orientation.x;
 local_goal.pose.orientation.y=goal.pose.orientation.y;
 local_goal.pose.orientation.z=goal.pose.orientation.z;
 local_goal.pose.orientation.w=goal.pose.orientation.w;
 //main difference
 local_goal.pose.position.x=gx;
 local_goal.pose.position.y=gy;
 return true;
}

int PipelinePlanner::CrossTorchArea(const double tx,const double ty,
const double sx,const double sy,const double ex,const double ey,
double& bx,double& by){
 // b=c+(n-c)*t, |bx-sx|=ax, |by-sy|=ay, 0<t<1 (condition to cross the boundary)
 //whether start point is in torch area or not
 if(!(std::abs(sx-tx)<=torch_area_x_&&std::abs(sy-ty)<=torch_area_y_)){
  return 1;
 };
 //whether end point is in torch area or not
 if(std::abs(ex-tx)<=torch_area_x_&&std::abs(ey-ty)<=torch_area_y_){
  return 0;
 };
 //assertion of valid length of line segment
 double len=sqrt(pow(ex-sx,2)+pow(ey-sy,2));
 if(len<tolerance_){
  return -2;
 };
 //calculation of the cross point
 //b=s+(e-s)*u, 0<u<1, |bx-tx|=ax or |by-ty|=ay (cross condition)
 double uxp,uxm,uyp,uym; double minu;
 if(std::abs(ex-sx)<tolerance_){
  uxp=2.0; uxm=2.0;
 } else {
  uxp=(tx-sx+torch_area_x_)/(ex-sx); uxm=(tx-sx-torch_area_x_)/(ex-sx);
  if(uxp<0.0)uxp=2.0; if(uxm<0.0)uxm=2.0;
 };
 if(std::abs(ey-sy)<tolerance_){
  uyp=2.0; uym=2.0;
 } else {
  uyp=(ty-sy+torch_area_y_)/(ey-sy); uym=(ty-sy-torch_area_y_)/(ey-sy);
  if(uyp<0.0)uyp=2.0; if(uym<0.0)uym=2.0;
 };
 minu=std::min(std::min(uxp,uxm),std::min(uyp,uym));
 if(minu>1.0){
  return -1;
 };
 bx=sx+(ex-sx)*minu; by=sy+(ey-sy)*minu;

 return 2;
}

double PipelinePlanner::TakeSegmentRadius(const double contained_radius){
 double radius;
 if(contained_radius==0.0){
  radius=pipe_radius_;
 } else {
  radius=contained_radius;
 };
 return radius;
}

#if MODE==MODE_GPU
void PipelinePlanner::ThreadNumberAdjusment(bool debug){
 cudaDeviceProp prop;
 int device=0;
 cudaError_t device_property_error;
 device_property_error=cudaGetDeviceProperties(&prop,device);
 if(device_property_error!=cudaSuccess){
  ROS_ERROR_STREAM("cudaGetDeviceProperties error: " << device_property_error);
 };
 if(debug){
  ROS_INFO_STREAM("max thread per block: " << prop.maxThreadsPerBlock);
 };
 if(num_threads_>prop.maxThreadsPerBlock){
  ROS_WARN("the number of thead you set is larger than maximum thread size");
  ROS_WARN_STREAM("the number of threads you set: " << num_threads_);
  ROS_WARN_STREAM("the maximum size of thread for this device: " << prop.maxThreadsPerBlock);

  num_threads_=prop.maxThreadsPerBlock;
  ROS_WARN_STREAM("we force to set the number of threads: " << num_threads_);
  ros::NodeHandle nh("~");
  nh.setParam("/move_base/PipelinePlanner/num_threads",prop.maxThreadsPerBlock);
 };
}
#endif


} // end namespace
