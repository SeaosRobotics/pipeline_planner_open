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
#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

int test_cuda(int a, int b){
 std::cout << "test cuda" << std::endl;
 int c;
 c=a+b;
 return c;
}

//__global__
//void calc_reference_values(const int numcheckpoints,const int numsegids,
//const double *sx,const double *sy,
//const double *radii,const double map_resolution,
//double *refper,double *difper,double *refpll,double *difpll){
// int ID=threadIdx.x;
// int width=blockDim.x;
// for(int i=ID;i<numsegids;i+=width){
//  int ci=i;
//  int ni=(ci+1)%numcheckpoints;
//  double normv=sqrt(pow(sx[ni]-sx[ci],2)+pow(sy[ni]-sy[ci],2));
//  double invnormv=1.0/normv;
//  refper[ci]=sx[ci]*(sy[ni]-sy[ci])-sy[ci]*(sx[ni]-sx[ci]); //reference value for perpendicular component
//  difper[ci]=radii[ci]*invnormv*(pow(sy[ni]-sy[ci],2)+pow(sx[ni]-sx[ci],2))
//               +map_resolution*(0.5* abs(sy[ni]-sy[ci])+0.5* abs(sx[ni]-sx[ci]));
//  refpll[ci]=(0.5*(sx[ni]+sx[ci]))*(sx[ni]-sx[ci])+(0.5*(sy[ni]+sy[ci]))*(sy[ni]-sy[ci]);//reference value for parallel component
//  //reference value for parallel component
//  difpll[ci]=(0.5*(sx[ni]-sx[ci]))*(sx[ni]-sx[ci])+(0.5*(sy[ni]-sy[ci]))*(sy[ni]-sy[ci])
//               +map_resolution*(0.5*abs(sx[ni]-sx[ci])+0.5*abs(sy[ni]-sy[ci]));
// };
//}
//
//__host__
//void calc_reference_values_cpu(const int numcheckpoints,const int numsegids,
//const double *sx,const double *sy,
//const double *radius,const double map_resolution,
//double *refper,double *difper,double *refpll,double *difpll){
// for(int i=0;i<numsegids;i++){
//  int ci=i;
//  int ni=(ci+1)%numcheckpoints;
//  double normv=sqrt(pow(sx[ni]-sx[ci],2)+pow(sy[ni]-sy[ci],2));
//  double invnormv=1.0/normv;
//  refper[ci]=sx[ci]*(sy[ni]-sy[ci])-sy[ci]*(sx[ni]-sx[ci]); //reference value for perpendicular component
//  difper[ci]=radius[ci]*invnormv*(pow(sy[ni]-sy[ci],2)+pow(sx[ni]-sx[ci],2))
//               +map_resolution*(0.5* abs(sy[ni]-sy[ci])+0.5* abs(sx[ni]-sx[ci]));
//  refpll[ci]=(0.5*(sx[ni]+sx[ci]))*(sx[ni]-sx[ci])+(0.5*(sy[ni]+sy[ci]))*(sy[ni]-sy[ci]);//reference value for parallel component
//  //reference value for parallel component
//  difpll[ci]=(0.5*(sx[ni]-sx[ci]))*(sx[ni]-sx[ci])+(0.5*(sy[ni]-sy[ci]))*(sy[ni]-sy[ci])
//               +map_resolution*(0.5*abs(sx[ni]-sx[ci])+0.5*abs(sy[ni]-sy[ci]));
// };
//}
//
//__global__
//void calcPipeline_device_main(const int numcheckpoints,const int numsegids,
//const double *sx,const double *sy,
//const double *radii,const double *sradii,const double *eradii,
//const double origin_x,const double origin_y,
//const unsigned int costsizex,const unsigned int costsizey,
//const double resolution,
//double *refper,double *difper,double *refpll,double *difpll,
//int *data){
// int ID=threadIdx.x;
// int width=blockDim.x;
// for(int i=ID;i<costsizex*costsizey;i+=width){
//  int idx=i%costsizex; int idy=i/costsizex;
//  double wx,wy;
//  wx=origin_x+(idx+0.5)*resolution;
//  wy=origin_y+(idy+0.5)*resolution;
//  for(int s=0;s<numsegids;s++){
//   int ci=s;
//   int ni=(ci+1)%numcheckpoints;
//   double curper=wx*(sy[ni]-sy[ci])-wy*(sx[ni]-sx[ci]);
//   double curpll=wx*(sx[ni]-sx[ci])+wy*(sy[ni]-sy[ci]);
//   //rectangular
//   if((abs(curper-refper[s])<difper[s])&&(abs(curpll-refpll[s])<difpll[s])){
//    data[i]=0;
//    break;
//   } else {
//    //start circle region
//    double distfromstart=sqrt(pow(wx-sx[ci],2.0)+pow(wy-sy[ci],2.0));
//    if(distfromstart<sradii[s]+resolution*0.5*sqrt(2.0)){
//     data[i]=0;
//     break;
//    };
//    //end circle region
//    double distfromend=sqrt(pow(wx-sx[ni],2.0)+pow(wy-sy[ni],2.0));
//    if(distfromend<eradii[s]+resolution*0.5*sqrt(2.0)){
//     data[i]=0;
//    };
//   };
//  };
// };
//}
//
//__host__
//void calcPipeline_device(const int numcheckpoints,
//const double *sx,const double *sy,const bool isClose,
//const double *radii,const double *sradii,const double *eradii,
//const double origin_x,const double origin_y,
//const unsigned int costsizex,const unsigned int costsizey,
//const double resolution,int *data,const int nthreads){
// int numsegids=(isClose==true)?numcheckpoints:numcheckpoints-1;
// //allocation
// double *refper,*difper,*refpll,*difpll;
// cudaMallocManaged(&refper,numsegids*sizeof(double));
// cudaMallocManaged(&difper,numsegids*sizeof(double));
// cudaMallocManaged(&refpll,numsegids*sizeof(double));
// cudaMallocManaged(&difpll,numsegids*sizeof(double));
// //preparation
// if(numsegids>31){
//  calc_reference_values<<<1,nthreads>>>(numcheckpoints,numsegids,sx,sy,
//    radii,resolution,refper,difper,refpll,difpll);
//  cudaDeviceSynchronize();
// } else {
//  calc_reference_values_cpu(numcheckpoints,numsegids,sx,sy,radii,resolution,
//    refper,difper,refpll,difpll);
// };
// //main calculation
// for(int i=0;i<costsizex*costsizey;i++){
//  data[i]=100;
// };
// calcPipeline_device_main<<<1,nthreads>>>(numcheckpoints,numsegids,
//   sx,sy,radii,sradii,eradii,origin_x,origin_y,costsizex,costsizey,resolution,
//   refper,difper,refpll,difpll,data);
// cudaDeviceSynchronize();
// //termination
// cudaFree(refper); cudaFree(difper); cudaFree(refpll); cudaFree(difpll);
//}

__global__
void DistanceFromCentreOnDevice(const int ns,
const double *stx,const double *sty,const double *stdev,
const unsigned int sizex,const unsigned int sizey,
const double resol, const double ox,const double oy,
double *dist_from_centre){
 int ID=threadIdx.x;
 int width=blockDim.x;
 for(int i=ID;i<sizex*sizey;i+=width){
  int idx=i%sizex; int idy=i/sizex;

  //We only calculate distance of a point in the pipeline
  if(dist_from_centre[i]==100.0)continue;

  double tmpdist,mindist;
  double px,py;
  px=ox+(idx+0.5)*resol;
  py=oy+(idy+0.5)*resol;
  for(int s=0;s<ns;s++){
   double cx=stx[s]; double cy=sty[s];
   double nx=stx[s+1]; double ny=sty[s+1];
   double cnx=nx-cx; double cny=ny-cy;
   double leng_cn=sqrt(pow(cnx,2)+pow(cny,2));
   double dirx=cnx/leng_cn; double diry=cny/leng_cn;
   //directional component
   double dcomp_c=cx*dirx+cy*diry;
   double dcomp_n=nx*dirx+ny*diry;
   double dcomp_p=px*dirx+py*diry;
   //perpendicular component
   double ppcx=ny-cy; double ppcy=-(nx-cx);
   double leng_ppc=sqrt(ppcx*ppcx+ppcy*ppcy);
   double perpx=ppcx/leng_ppc; double perpy=ppcy/leng_ppc;
   //conditions
   if(dcomp_p>dcomp_n){
    //double npx=px-nx;double npy=py-ny;
    double npx=px-(nx+perpx*stdev[s]);
    double npy=py-(nx+perpy*stdev[s]);
    tmpdist=sqrt(npx*npx+npy*npy);
   } else if(dcomp_c>dcomp_p){
    //double cpx=px-cx;double cpy=py-cy;
    double cpx=px-(cx+perpx*stdev[s]);
    double cpy=py-(cy+perpy*stdev[s]);
    tmpdist=sqrt(cpx*cpx+cpy*cpy);
   } else {
//    //perpendicular component
//    double ppcx=ny-cy; double ppcy=-(nx-cx);
//    double leng_ppc=sqrt(ppcx*ppcx+ppcy*ppcy);
//    double perpx=ppcx/leng_ppc; double perpy=ppcy/leng_ppc;
    //double pcomp_c=cx*perpx+cy*perpy;
    double pcomp_c=cx*perpx+cy*perpy+stdev[s];
    double pcomp_p=px*perpx+py*perpy;
    tmpdist=abs(pcomp_c-pcomp_p);
   };
   if(s==0){
    mindist=tmpdist;
   } else {
    if(tmpdist<mindist)mindist=tmpdist;
   };
  };
  dist_from_centre[i]=mindist;
 };

}

__host__
void DistanceFromCentreCalling(const int numstones,
const double *stonex,const double *stoney,const double *stonedev,
const unsigned int costsizex,const unsigned int costsizey,
const double map_resolution, const double origin_x,const double origin_y,
double *dist_from_centre,
const int nthreads,
const bool debug){

 DistanceFromCentreOnDevice<<<1,nthreads>>>(numstones,stonex,stoney,stonedev,
 costsizex,costsizey,map_resolution, origin_x,origin_y,dist_from_centre);
 cudaDeviceSynchronize();

}

__global__
void prepare_daub(const int numstones,const double *sx,const double *sy,
const double *radius,const double map_resolution,
const double startx,const double starty,
double *refper,double *difper,double *refpll,double *difpll){
 int ID=threadIdx.x;
 int width=blockDim.x;
 for(int i=ID;i<numstones;i+=width){
  double normv=sqrt(pow(sx[i+1]-sx[i],2)+pow(sy[i+1]-sy[i],2));
  double invnormv=1.0/normv;
  refper[i]=sx[i]*(sy[i+1]-sy[i])-sy[i]*(sx[i+1]-sx[i]); //reference value for perpendicular component
  difper[i]=radius[i]*invnormv*(pow(sy[i+1]-sy[i],2)+pow(sx[i+1]-sx[i],2))
               +map_resolution*(0.5* abs(sy[i+1]-sy[i])+0.5* abs(sx[i+1]-sx[i]));
  refpll[i]=(0.5*(sx[i+1]+sx[i]))*(sx[i+1]-sx[i])+(0.5*(sy[i+1]+sy[i]))*(sy[i+1]-sy[i]);//reference value for parallel component
  //reference value for parallel component
  difpll[i]=(0.5*(sx[i+1]-sx[i]))*(sx[i+1]-sx[i])+(0.5*(sy[i+1]-sy[i]))*(sy[i+1]-sy[i])
               +map_resolution*(0.5*abs(sx[i+1]-sx[i])+0.5*abs(sy[i+1]-sy[i]));
 };
}

__host__
void prepare_daub_cpu(const int numstones,const double *sx,const double *sy,
const double *radius,const double map_resolution,
const double startx,const double starty,
double *refper,double *difper,double *refpll,double *difpll){
 for(int i=0;i<numstones;i++){
  double normv=sqrt(pow(sx[i+1]-sx[i],2)+pow(sy[i+1]-sy[i],2));
  double invnormv=1.0/normv;
  refper[i]=sx[i]*(sy[i+1]-sy[i])-sy[i]*(sx[i+1]-sx[i]); //reference value for perpendicular component
  difper[i]=radius[i]*invnormv*(pow(sy[i+1]-sy[i],2)+pow(sx[i+1]-sx[i],2))
               +map_resolution*(0.5* abs(sy[i+1]-sy[i])+0.5* abs(sx[i+1]-sx[i]));
  refpll[i]=(0.5*(sx[i+1]+sx[i]))*(sx[i+1]-sx[i])+(0.5*(sy[i+1]+sy[i]))*(sy[i+1]-sy[i]);//reference value for parallel component
  //reference value for parallel component
  difpll[i]=(0.5*(sx[i+1]-sx[i]))*(sx[i+1]-sx[i])+(0.5*(sy[i+1]-sy[i]))*(sy[i+1]-sy[i])
               +map_resolution*(0.5*abs(sx[i+1]-sx[i])+0.5*abs(sy[i+1]-sy[i]));
 };

}

__global__
void daub_costmap(const int size_x,const int size_y,
const int min_index_x,const int max_index_x,
const int min_index_y,const int max_index_y,
const double map_resolution,
const double origin_x,const double origin_y,const int numstones,
const double startx,const double starty,
const double *sx,const double *sy,
const double *sradius,const double *eradius,
const double *refper,const double *difper,const double *refpll,const double *difpll,
const double sedgepll,const double eedgepll,const double startpll,
const int start_area,
int *daub){
 int ID=threadIdx.x;
 int width=blockDim.x;
 //for(int i=ID;i<size_x*size_y;i+=width)
 for(int li=ID;li<(max_index_x-min_index_x)*(max_index_y-min_index_y);li+=width){
  //int idx=i%size_x; int idy=i/size_x;
  int idx=li%(max_index_x-min_index_x)+min_index_x;
  int idy=li/(max_index_x-min_index_x)+min_index_y;
  int i=idy*size_x+idx;
  double wx,wy;
  wx=origin_x+(idx+0.5)*map_resolution;
  wy=origin_y+(idy+0.5)*map_resolution;
  for(int s=0;s<numstones;s++){
   double curper=wx*(sy[s+1]-sy[s])-wy*(sx[s+1]-sx[s]);
   double curpll=wx*(sx[s+1]-sx[s])+wy*(sy[s+1]-sy[s]);
   //pipeline rectangular
   if((abs(curper-refper[s]) < difper[s])&&(abs(curpll-refpll[s]) < difpll[s])){
    if((s==0)&&((start_area==2
      &&((curpll-sedgepll)*(startpll-curpll)>0)
      &&(abs(curpll-startpll)>0.1))
      ||(start_area==3))){
     daub[i]=2;
     break;
    } else {
     daub[i]=0;
    };
   };
   //pipelince connection circle
   double distancefromconnection; double startfromconnection;
   //start edge circle
   distancefromconnection=sqrt((wx-sx[s])*(wx-sx[s])+(wy-sy[s])*(wy-sy[s]));
   if(distancefromconnection<sradius[s]+map_resolution*(0.5*sqrt(2.0))){
    //we should set condition to plug start circle
    startfromconnection=sqrt((startx-sx[s])*(startx-sx[s])+
        (starty-sy[s])*(starty-sy[s]));
    if((s==0)&&start_area==1
       &&((sedgepll-curpll)*(eedgepll-sedgepll)>0)
       &&(distancefromconnection>(startfromconnection+sradius[s])*0.5)){
     daub[i]=2;
     break;
    } else {
     daub[i]=0;
    };
   };
   //end edge circle
   distancefromconnection=sqrt((wx-sx[s+1])*(wx-sx[s+1])+(wy-sy[s+1])*(wy-sy[s+1]));
   if(distancefromconnection<eradius[s]+map_resolution*(0.5*sqrt(2.0))){
    daub[i]=0;
   };
  };
 };
}

__global__
void sloped_cost(const int size_x,const int size_y,
const int min_index_x,const int max_index_x,
const int min_index_y,const int max_index_y,
const int *daub,
const double centre_weight,const double *dist_from_pipe_centre,
unsigned char lethal,
unsigned char *newCost){
 int ID=threadIdx.x;
 int width=blockDim.x;
 //for(int i=ID;i<size_x*size_y;i+=width)
 for(int li=ID;li<(max_index_x-min_index_x)*(max_index_y-min_index_y);li+=width){
  int idx=li%(max_index_x-min_index_x)+min_index_x;
  int idy=li/(max_index_x-min_index_x)+min_index_y;
  int i=idy*size_x+idx;
  if(daub[i]==1||daub[i]==2){
   newCost[i]=lethal;
  } else {
   if((newCost[i]!=253)&&(newCost[i]!=254)&&(newCost[i]!=255)){
    newCost[i]+=(unsigned char)(centre_weight*dist_from_pipe_centre[i]);
   };
  };
 };
}

__host__
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
const bool debug){

 cudaError_t error_malloc;
 //allocation of daub
 int *daub;
 if(debug){
  error_malloc=cudaMallocManaged(&daub,(costsizex*costsizey)*sizeof(int));
  std::cout << "cudaMallocManaged daub: " << error_malloc << std::endl;
 } else {
  cudaMallocManaged(&daub,(costsizex*costsizey)*sizeof(int));
 };
 for(int i=0;i<costsizex*costsizey;i++){
  daub[i]=1;
 };
 //allocation of variable needed for pipesegment calculation
 double *refper,*difper,*refpll,*difpll;
 if(debug){
  error_malloc=cudaMallocManaged(&refper,numstones*sizeof(double));
  std::cout << "cudaMallocManaged refper: " << error_malloc << std::endl;
  error_malloc=cudaMallocManaged(&difper,numstones*sizeof(double));
  std::cout << "cudaMallocManaged difper: " << error_malloc << std::endl;
  error_malloc=cudaMallocManaged(&refpll,numstones*sizeof(double));
  std::cout << "cudaMallocManaged refpll: " << error_malloc << std::endl;
  error_malloc=cudaMallocManaged(&difpll,numstones*sizeof(double));
  std::cout << "cudaMallocManaged difpll: " << error_malloc << std::endl;
 } else {
  cudaMallocManaged(&refper,numstones*sizeof(double));
  cudaMallocManaged(&difper,numstones*sizeof(double));
  cudaMallocManaged(&refpll,numstones*sizeof(double));
  cudaMallocManaged(&difpll,numstones*sizeof(double));
 };
 //setting of GPU calculation
 //if(true);
 if(numstones>31){
  //use GPU
  //calling of GPU function
  prepare_daub<<<1,nthreads>>>(numstones,stonex,stoney,radius,
    map_resolution,startx,starty,refper,difper,refpll,difpll);
  //synchronisation
  cudaDeviceSynchronize();
 } else {
  //don't use GPU
  //calling of CPU calculation
  prepare_daub_cpu(numstones,stonex,stoney,radius,
    map_resolution,startx,starty,refper,difper,refpll,difpll);
 };

 //variables for the start position
 double sedgepll,eedgepll,startpll;
 int start_area=0;
 sedgepll=(stonex[0])*(stonex[1]-stonex[0])+(stoney[0])*(stoney[1]-stoney[0]);//parallel component for the start edge
 eedgepll=(stonex[1])*(stonex[1]-stonex[0])+(stoney[1])*(stoney[1]-stoney[0]);//parallel component for the start edge
 startpll=(startx)*(stonex[1]-stonex[0])+(starty)*(stoney[1]-stoney[0]);//parallel component for the start point
 if((sedgepll-startpll)*(eedgepll-sedgepll)>0){
  start_area=1;//start point is out from start edge
 } else if((startpll-sedgepll)*(eedgepll-startpll)>0){
  start_area=2;//start point is in the first pipe segment
 } else {
  start_area=3;//start point is out from end edge
 };

 //calling of pipesegments calculation
 if(debug){
  std::cout << "CUDA nthreads:" << nthreads << std::endl;
  std::cout << "CUDA costsizex*costsizey:" << costsizex*costsizey << std::endl;
 };
 daub_costmap<<<1,nthreads>>>(costsizex,costsizey,
   min_index_x,max_index_x,min_index_y,max_index_y,map_resolution,
   origin_x,origin_y,numstones,startx,starty,stonex,stoney,sradius,eradius,
   refper,difper,refpll,difpll,
   sedgepll,eedgepll,startpll,start_area,daub);

 //synchronisation
 cudaDeviceSynchronize();

 //new cost function
 //calling of cuda function
 sloped_cost<<<1,nthreads>>>(costsizex,costsizey,
    min_index_x,max_index_x,min_index_y,max_index_y,
    daub,centre_weight,
    dist_from_centre,lethal,newCost);

 //synchronise
 cudaDeviceSynchronize();

 //free allocated variables
 cudaError_t error_free;
 if(debug){
  error_free=cudaFree(daub);
  std::cout << "free daub: " << error_free << std::endl;
  error_free=cudaFree(refper);
  std::cout << "free refper: " << error_free << std::endl;
  error_free=cudaFree(difper);
  std::cout << "free difper: " << error_free << std::endl;
  error_free=cudaFree(refpll);
  std::cout << "free refpll: " << error_free << std::endl;
  error_free=cudaFree(difpll);
  std::cout << "free difpll: " << error_free << std::endl;
 } else {
  cudaFree(daub);
  cudaFree(refper);
  cudaFree(difper);
  cudaFree(refpll);
  cudaFree(difpll);
 };

 return;
}
