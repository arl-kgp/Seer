//
// adapted from ros example and april tag examples - palash
//
#include <fstream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libconfig.h++>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include <Eigen/Dense>
#include "popt_pp.h"
#include "seer/AprilTag.h"      // rosmsg
#include "seer/AprilTagList.h"  // rosmsg
#include "rigidtransform.h"

using std::ofstream;
using namespace cv;
using namespace std;
using namespace libconfig;

static const std::string OPENCV_WINDOW = "Image window";

const double PI = 3.14159265358979323846;
const double TWOPI = 2.0 * PI;

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t + PI, TWOPI) - PI;
  } else {
    t = fmod(t - PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch,
                  double& roll) {
  yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
  roll = standardRad(
      atan2(wRo(0, 2) * s - wRo(1, 2) * c, -wRo(0, 1) * s + wRo(1, 1) * c));
}

class AprilTagNode {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher odom_pub_;
  // ros::Publisher tag_list_pub;
  AprilTags::TagDetector* tag_detector;

  // allow configurations for these:
  AprilTags::TagCodes tag_codes;
  double camera_focal_length_x;  // in pixels. late 2013 macbookpro retina = 700
  double camera_focal_length_y;  // in pixels
  double tag_size;               // tag side length of frame in meters
  bool show_debug_image;
  vector<Eigen::Vector3d> P1s, P2s,P2s_calc;

  /*int i,j;
  config_t cfg,*cf;
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      config_setting_t *R = Eigen::Matrix3d::Constant(i,j,1.0);
  config_setting_t *T = {0,0,0};
  cf = &cfg;
  config_init(cf);*/

  // cv::Mat imgL;
  // cv::Mat imgR;

 public:
  AprilTagNode()
      : it_(nh_),
        tag_codes(AprilTags::tagCodes36h11),
        tag_detector(NULL),
        camera_focal_length_y(414),
        camera_focal_length_x(414),
        tag_size(0.015),  // 1 1/8in marker = 0.029m
        show_debug_image(false) {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ =
    //    it_.subscribe("/usb_cam/image_raw", 1, &AprilTagNode::imageCb, this);
    image_pub_ = it_.advertise("/seer_debug/output_video", 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ground_truth", 1);
    // tag_list_pub = nh_.advertise<seer::AprilTagList>("/seers", 100);

    // Use a private node handle so that multiple instances of the node can
    // be run simultaneously while using different parameters.
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<double>("focal_length_px", camera_focal_length_x,
                                      700.0);
    private_node_handle.param<double>("tag_size_cm", tag_size, 2.9);
    private_node_handle.param<bool>("show_debug_image", show_debug_image,
                                    false);

    camera_focal_length_y = camera_focal_length_x;  // meh
    tag_size = tag_size / 100.0;  // library takes input in meters

    cout << "got focal length " << camera_focal_length_x << endl;
    cout << "got tag size " << tag_size << endl;
    tag_detector = new AprilTags::TagDetector(tag_codes);

    if (show_debug_image) {
      cv::namedWindow(OPENCV_WINDOW);
    }
    ROS_INFO("Done");
  }
  int m_mode;

  ~AprilTagNode() {
    if (show_debug_image) {
      cv::destroyWindow(OPENCV_WINDOW);
      delete tag_detector;
    }
  }

  float Eucl_distance(Eigen::Vector3d v1,Eigen::Vector3d v2){
    return sqrt(pow((v1[0]-v2[0]),2)+pow((v1[1]-v2[1]),2)+pow((v1[2]-v1[2]),2));
  }
  void calibrate(char key, Mat& image_grayL, Mat& image_grayR) {
    vector<AprilTags::TagDetection> detectionsL =
        AprilTagNode::tag_detector->extractTags(image_grayL);
    vector<seer::AprilTag> tag_msgs;
    vector<AprilTags::TagDetection> detectionsR =
        tag_detector->extractTags(image_grayR);
    //  vector<seer::AprilTag> tag_msgs;
    if (detectionsR.size() > 0 && detectionsL.size() > 0) {
      Eigen::Vector3d translationL, translationR;
      Eigen::Matrix3d rotationL, rotationR;

      static const char *file ="T_matrix.cfg";
      Config cfg;
      Setting &root=cfg.getRoot();

      if(!root.exists("Matrix"))
        root.add("Matrix",Setting::TypeGroup);

      Setting &Matrix=root["Matrix"];

      if (key != 27) {
        //for (size_t i = 0; i < detectionsL.size(); i++) {
          detectionsL[0].getRelativeTranslationRotation(
              tag_size, camera_focal_length_x, camera_focal_length_y,
              image_grayL.cols / 2, image_grayL.rows / 2, translationL,
              rotationL);

          P1s.push_back(Eigen::Vector3d(translationL(1), -translationL(2),
                                        2.89698162598 - translationL(0)));
        //}
        //for (size_t i = 0; i < detectionsR.size(); i++) {
          detectionsR[0].getRelativeTranslationRotation(
              tag_size, camera_focal_length_x, camera_focal_length_y,
              image_grayR.cols / 2, image_grayR.rows / 2, translationR,
              rotationR);
          P2s.push_back(Eigen::Vector3d(translationR(1), -translationR(2),
                                        2.89698162598 - translationR(0)));
        //}
        ROS_INFO("Grabbed a set of points");
      } else {
        TransformType RT = computeRigidTransform(P1s, P2s);
      //  if(config_read_file(&cfg,"T_matrix")==CONFIG_TRUE){
          std::cout << RT.first << endl;
          std::cout << (RT.second)[0] << "  " << (RT.second)[1] << "  "
                    << (RT.second)[2] << endl;
          cout << endl;
        /*  R=config_lookup(&cfg,"matrix.R");
          T=config_lookup(&cfg,"matrix.T");
          config_setting_add(R,RT.first);
          config_setting_add(T,RT.second);
          config_write_file(&cfg,"T_matrix");*/
          Setting &R1=Matrix.add("R1",Setting::TypeList);
          Setting &R2=Matrix.add("R2",Setting::TypeList);
          Setting &R3=Matrix.add("R3",Setting::TypeList);
          Setting &T=Matrix.add("T",Setting::TypeList);
              for(int i=0;i<3;i++){
            R1.add(Setting::TypeFloat) = (RT.first)(0,i);
            R2.add(Setting::TypeFloat) = (RT.first)(1,i);
            R3.add(Setting::TypeFloat) = (RT.first)(2,i);
            T.add(Setting::TypeFloat) = (RT.second)[i];
          }

          cfg.writeFile(file);
          cerr<<"New Configuration successfully written to: "<<file <<endl;

          P1s.clear();
          P2s.clear();
        }
      }
      return;
      //exit(EXIT_FAILURE);
    }

    void normal(char key, Mat& image_grayL, Mat& image_grayR) {
      vector<AprilTags::TagDetection> detectionsL =
          AprilTagNode::tag_detector->extractTags(image_grayL);
      vector<seer::AprilTag> tag_msgs;
      vector<AprilTags::TagDetection> detectionsR =
          tag_detector->extractTags(image_grayR);
      //  vector<seer::AprilTag> tag_msgs;
      if (detectionsR.size() > 0 && detectionsL.size() > 0) {
        Eigen::Vector3d translationL, translationR;
        Eigen::Matrix3d rotationL, rotationR;

        static const char *file ="/home/pikachu/catkin_ws/T_matrix.cfg";
        Config cfg;
        try{
          cfg.readFile(file);
        }
        catch(const FileIOException &fioex)
        {
          std::cerr<<"I/O error while reading file"<<endl;
          return;
        }
        //Setting& root=cfg.getRoot();
        //Setting &Matrix=root["Matrix"];

        if (key != 27) {
          //for (size_t i = 0; i < detectionsL.size(); i++) {
            detectionsL[0].getRelativeTranslationRotation(
                tag_size, camera_focal_length_x, camera_focal_length_y,
                image_grayL.cols / 2, image_grayL.rows / 2, translationL,
                rotationL);

            P1s.push_back(Eigen::Vector3d(translationL(1), -translationL(2),
                                          2.89698162598 - translationL(0)));
          //}
          //for (size_t i = 0; i < detectionsR.size(); i++) {
            detectionsR[0].getRelativeTranslationRotation(
                tag_size, camera_focal_length_x, camera_focal_length_y,
                image_grayR.cols / 2, image_grayR.rows / 2, translationR,
                rotationR);
            P2s.push_back(Eigen::Vector3d(translationR(1), -translationR(2),
                                          2.89698162598 - translationR(0)));
          //}
          ROS_INFO("Grabbed a set of points");
        } else {
            vector<float> r1,r2,r3,t;
            float error;
            //const char* var3;
            Eigen::Matrix3d R;
            Eigen::Vector3d T;
            /*if(cfg.exists("Matrix.R1")&&cfg.exists("Matrix.R2")&&cfg.exists("Matrix.R3")&&cfg.exists("Matrix.T"))
              cout<<"Exists"<<endl;
            //Setting &Matrix=root["Matrix"];
          // if((cfg.lookupValue("Matrix.R1",*r1)) && (cfg.lookupValue("Matrix.R2",*r2)) && (cfg.lookupValue("Matrix.R3",*r2))
            //   && (cfg.lookupValue("Matrix.T",*t))){
           /*t=cfg.lookup("Matrix.T");
           r1[3]=cfg.lookup("Matrix.R1");
           r2[3]=cfg.lookup("Matrix.R2");
           r3[3]=cfg.lookup("Matrix.R3");*/
           const Setting &R1_set=cfg.lookup("Matrix.R1");
           const Setting &R2_set=cfg.lookup("Matrix.R2");
           const Setting &R3_set=cfg.lookup("Matrix.R3");
           const Setting &T_set=cfg.lookup("Matrix.T");
           for(int n=0 ; n < R1_set.getLength(); n++){
             r1.push_back(R1_set[n]);
             r2.push_back(R2_set[n]);
             r3.push_back(R3_set[n]);
             t.push_back(T_set[n]);
           }
           //std::cout << r1[0] << '\n';
            for(int i=0;i<3;i++){
              T[i]=t[i];//cfg.lookup("Matrix.T");
              for(int j=0;j<3;j++){
                if(i==0) R(i,j) = r1[j];//cfg.lookup("Matrix.R1");
                if(i==1) R(i,j) = r2[j];//cfg.lookup("Matrix.R2");
                if(i==2) R(i,j) = r3[j];//cfg.lookup("Matrix.R3");
                //cout<<"hi"<<endl;
              }
            }
              Eigen::Vector3d V;
              for(size_t i=0;i<P1s.size();i++){
                V=R*P1s[i]+T;
                //cout<<V[0]<<endl;
              P2s_calc.push_back(V);
              //cout<<P1s[i]<<" :calc: "<<P2s_calc[i]<<endl;
            }
              for(size_t i=0;i<P2s.size();i++){
                error=Eucl_distance(P2s_calc[i],P2s[i]);
                cerr<<"Error for "<< i+1 << "Point is:"<<error<<endl;
            }

            cout << endl;
          //  P1s.clear();
            //P2s.clear();
          }
        //  cout<<"222"<<endl;
        //}
      }
      //exit(EXIT_FAILURE);
      return;
    }


  void imgCallback(const sensor_msgs::ImageConstPtr& msg_left,
                   const sensor_msgs::ImageConstPtr& msg_right) {
    cv::Mat tmpL, tmpR, image_grayL, image_grayR;
    tmpL = cv_bridge::toCvShare(msg_left, "bgr8")->image;
    tmpR = cv_bridge::toCvShare(msg_right, "bgr8")->image;
    if (tmpL.empty() || tmpR.empty()) return;

    if (m_mode==1){
	    cv::cvtColor(tmpL, image_grayL, CV_BGR2GRAY);
	    cv::cvtColor(tmpR, image_grayR, CV_BGR2GRAY);

	    imshow("Left_Input_Calibration", image_grayL);
	    imshow("Right_Input_Calibration", image_grayR);
	    char key = (char)cv::waitKey(30);
	    if (key > 0) {
	      //calibrate(key, image_grayL, image_grayR);
        normal(key, image_grayL, image_grayR);
	    }
      exit(EXIT_FAILURE);
    }
    else if(m_mode == 0){
      cv::cvtColor(tmpL, image_grayL, CV_BGR2GRAY);
	    cv::cvtColor(tmpR, image_grayR, CV_BGR2GRAY);

	    imshow("Left_Input", image_grayL);
	    imshow("Right_Input", image_grayR);
	    char key = (char)cv::waitKey(30);
	    if (key > 0) {
	      calibrate(key, image_grayL, image_grayR);
	    }
      exit(EXIT_FAILURE);
    }
  }
  //config_destroy(cf);
};

/*************************************************    MAIN
 * ************************************************/

/*
* TODO:
* Write R and T to file.
* Read R and T from file.
* Read all hard coded values from lib config file
* Calculate RMSE of calibration and display with the R and T cout
* Write Mode 2: Detect april tags and simply cout euclidean distance
*/


int main(int argc, char** argv) {
  ros::init(argc, argv, "seer_node");
  ros::NodeHandle nh;
  AprilTagNode atn;
  int mode,m_mode;
  static struct poptOption options[] = {
      {"mode", 'm', POPT_ARG_INT, &mode, 0,
       "Set m=1 for calibration, m=0 for testing", "NUM"},
      POPT_AUTOHELP{NULL, 0, 0, NULL, 0, NULL, NULL}};
  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while ((c = popt.getNextOpt()) >= 0) {
  }
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(
      nh, "/usb_cam/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(
      nh, "/usb_cam2/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left,
                                                 sub_img_right);
  atn.m_mode = mode;
  sync.registerCallback(boost::bind(&AprilTagNode::imgCallback, atn, _1, _2));



  ros::spin();
  return 0;
}
