#include <stdexcept>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Int8.h>

#include <math.h>
#include <Eigen/Geometry>

class Vref
{
public:
  explicit Vref (int queueSize)
    : nodeHandle_ (""),
      nodeHandlePrivate_ ("~"),
      queueSize_ (queueSize),
      objectPositionFilteredSubscriber_ (),
      vrefPublisher_ (),
      rotationToObjectRef_(0.0),//rad
      distanceToObjectRef_(0.0),//m
      minDistanceToObjectRef_(0.7),//m param
      kRotInitThreshMaxDeg_(15.0),//degree param
      rotInitMaxVel_(0.2),//rad/s param
      kRotInit_(0.0),//1/s
      vRefRotInitDeadZone_(5.0),//deg param
      commandMode_(0),
      objectPose_()
  {
    typedef boost::function<void (const geometry_msgs::PoseStampedConstPtr&)>
      callback_t;

    typedef boost::function<void (const std_msgs::Int8ConstPtr&)>
      callbackBciCommand_t;

    kRotInit_ = rotInitMaxVel_*180/(kRotInitThreshMaxDeg_*M_PI);

    callback_t callback =
      boost::bind (&Vref::callback, this, _1);

    callbackBciCommand_t callbackBciCommand =
      boost::bind (&Vref::callbackBciCommand, this, _1);

    objectPositionFilteredSubscriber_ =
      nodeHandle_.subscribe
      ("object_position_filtered", queueSize_,
       callback);

    bciCommandSubscriber_ =
      nodeHandle_.subscribe
      ("bci_command", queueSize_,
       callbackBciCommand);

    vrefPublisher_ =
      nodeHandle_.advertise<geometry_msgs::Vector3Stamped>
      ("velocity", queueSize_);
  }

  ~Vref ()
  {
  }

  void spin ()
  {
    ros::Rate rate(50);
    while (ros::ok())
      {
	ros::spinOnce();
	rate.sleep();
      }
  }

  void computeRotationToObjectRef(const geometry_msgs::PoseStamped& objectPosition, double& rotationToObjectRef)
  {

    rotationToObjectRef = atan2( objectPosition.pose.position.y, objectPosition.pose.position.x);
  }

  void computeDistanceToObjectRef(const geometry_msgs::PoseStamped& objectPosition, double& distanceToObjectRef)
  {
    distanceToObjectRef = sqrt(objectPosition.pose.position.y*objectPosition.pose.position.y+\
                               objectPosition.pose.position.x*objectPosition.pose.position.x);
  }

  void computeVRefRotInit(const double& rotationToObjectRef, double& vRefRot)
  {
    double vrefRotTemp = 0.0;
    //deadzone of rotationToObjectRef
    //if the error is small there is no need to move
    if( abs(rotationToObjectRef) > (vRefRotInitDeadZone_*M_PI/180))
    {
        vrefRotTemp = -1*kRotInit_*rotationToObjectRef;
        //gap the result
        if(vrefRotTemp > rotInitMaxVel_)
        {
            vrefRotTemp = rotInitMaxVel_;
        }
        else
        {
        }

        if(vrefRotTemp < -1*rotInitMaxVel_)
        {
            vrefRotTemp = -1*rotInitMaxVel_;
        }
        else
        {
        }
    }
    else
    {
        vrefRotTemp = 0.0;
    } 
    vRefRot = vrefRotTemp;
  }



//objectPosition is the pose in the waist of the robot
  void callback (const geometry_msgs::PoseStampedConstPtr& objectPosition)
  {
        Eigen::Quaterniond objectPositionInCamQuat(objectPosition->pose.orientation.w,
                                           objectPosition->pose.orientation.x,
                                           objectPosition->pose.orientation.y,
                                           objectPosition->pose.orientation.z);
    
        Eigen::Matrix3d objectPositionRotInCam;

        objectPositionRotInCam = objectPositionInCamQuat.toRotationMatrix();

        Eigen::Matrix4d objectPositionInCam;

//copy the rotation part
        for(int i = 0; i < 3; i++)
        {
            for(int j  = 0; j < 3; j++ )
            {
                objectPositionInCam(i,j)= objectPositionRotInCam.coeff(i,j);
            }
        }

//copy the translation part

        objectPositionInCam(0,3) = objectPosition->pose.position.x;
        objectPositionInCam(1,3) = objectPosition->pose.position.y;
        objectPositionInCam(2,3) = objectPosition->pose.position.z;
        objectPositionInCam(3,3) = 1;
        objectPositionInCam(3,0) = 0;
        objectPositionInCam(3,1) = 0;
        objectPositionInCam(3,2) = 0;

//head_to cam_xtion_rgb
//[ -2.44122110e-02, -1.89231351e-01, 9.81629014e-01, 8.69229361e-02, 
//  -9.99699354e-01, 2.36575282e-03, -2.44055502e-02, 1.49334883e-02, 
//   2.29600375e-03, -9.81929660e-01, -1.89232215e-01, 1.08828329e-01, 
//   0., 0., 0., 1. ]

//change the reference from the xtion camera to the head
        Eigen::Matrix4d tranformXtionRGBToHead;
        tranformXtionRGBToHead << -2.44122110e-02, -1.89231351e-01, 9.81629014e-01, 8.69229361e-02,\
                                  -9.99699354e-01, 2.36575282e-03, -2.44055502e-02, 1.49334883e-02,\
                                   2.29600375e-03, -9.81929660e-01, -1.89232215e-01, 1.08828329e-01,\
                                   0., 0., 0., 1.;

//change the reference from the head to the waist
        Eigen::Matrix4d tranformHeadToWaist;     
        tranformHeadToWaist << 0.92106099400288488, -2.1800758211874719e-30, 0.38941834230865091, 0.02499999999999997,\
                               2.0079828028645883e-30, 1.0, 8.4896151239399621e-31, -1.5715137206047549e-32,\
                               -0.38941834230865091, 6.9383317455266251e-48, 0.92106099400288488, 0.64800000000000002,\
                               0.0, 0.0, 0.0, 1.0;

        Eigen::Matrix4d objectPositionInWaist;

        objectPositionInWaist = tranformHeadToWaist * (tranformXtionRGBToHead * objectPositionInCam);
//robot.geom.signal('head2').value
//update the object position
//((0.92106099400288488, -2.1800758211874719e-30, 0.38941834230865091, 0.02499999999999997), (2.0079828028645883e-30, 1.0, 8.4896151239399621e-31, -1.5715137206047549e-32), (-0.38941834230865091, 6.9383317455266251e-48, 0.92106099400288488, 0.64800000000000002), (0.0, 0.0, 0.0, 1.0))

//FIXME update the quaternion too

        objectPose_.pose.position.x = objectPositionInWaist.coeff(0,3);
        objectPose_.pose.position.y = objectPositionInWaist.coeff(1,3);
        objectPose_.pose.position.z = objectPositionInWaist.coeff(2,3);
    
        objectPose_.header = objectPosition->header;
  }

  void callbackBciCommand(const std_msgs::Int8ConstPtr& bciCommand)
  {

//FIXME send the idle walk command or 
//the last command or 
//approximate the position of the object thanks to the last command sent
//and send the appropriate command (with the guessed object position)
 
    int bciCommandReceived = 0;
    bciCommandReceived = int(bciCommand->data);
     //update the mode of control
     //commandMode:
     //0 idle walk
     //1 walk forward
     //2 circle around by the right
     //4 circle around by the left
     //5 start/stop assistive navigation

    computeRotationToObjectRef( objectPose_, rotationToObjectRef_);
    computeDistanceToObjectRef( objectPose_, distanceToObjectRef_);

//copy and send the velocity command
    geometry_msgs::Vector3StampedPtr vref =
      boost::make_shared<geometry_msgs::Vector3Stamped> ();

// check the control mode set by the callback bci _command
    switch(bciCommandReceived)
    {
        case 0:
        {    
            //idle walk with (with orientation correction?)
            double vrefRotComputed = 0.0;
            computeVRefRotInit(rotationToObjectRef_, vrefRotComputed);

            vref->header = objectPose_.header;
            vref->vector.x = 0.001;
            vref->vector.y = 0.;
            vref->vector.z = vrefRotComputed;
            break;
        }
        case 1:
        {
        //walk forward (with orientation corection?) and stop at minDistanceToObjectRef_
            double vrefRotComputed = 0.0;
            computeVRefRotInit(rotationToObjectRef_, vrefRotComputed);

            if(distanceToObjectRef_ > minDistanceToObjectRef_)
            {
                vref->header = objectPose_.header;
                vref->vector.x = 0.1;
                vref->vector.y = 0.;
                vref->vector.z = vrefRotComputed;
            }
            else
            {
                vref->header = objectPose_.header;
                vref->vector.x = 0.001;
                vref->vector.y = 0.;
                vref->vector.z = vrefRotComputed;
            }
 
            break;
        }
        case 2:
        {
        //circle around by the ritght
                double alpha = 0.087; //rad
                double dotAlpha = alpha/0.8; //rad/s 0.8 is th eperiod of 1 step
                double dotY = distanceToObjectRef_*dotAlpha*sin(alpha-(M_PI/2));
                double dotXForward = distanceToObjectRef_*dotAlpha*cos(alpha-(M_PI/2));

                vref->header = objectPose_.header;
                vref->vector.x = dotXForward;
                vref->vector.y = dotY;
                vref->vector.z = dotAlpha;
            break;
        }
        case 3:
        {
        //nothing to do
            break;
        }
        case 4:
        {
        //circle around by the left
                double alpha = -0.087; //rad
                double dotAlpha = alpha/0.8; //rad/s 0.8 is th eperiod of 1 step
                double dotY = distanceToObjectRef_*dotAlpha*sin(alpha-(M_PI/2));
                double dotXForward = distanceToObjectRef_*dotAlpha*cos(alpha-(M_PI/2));

                vref->header = objectPose_.header;
                vref->vector.x = dotXForward;
                vref->vector.y = dotY;
                vref->vector.z = dotAlpha;
            break;
        }
        case 5:
        {
        //start/stop assistive navigation
            break;
        }
        default:
        {
        //idle walk with (with orientation correction?)
            double vrefRotComputed = 0.0;
            computeVRefRotInit(rotationToObjectRef_, vrefRotComputed);

            vref->header = objectPose_.header;
            vref->vector.x = 0.001;
            vref->vector.y = 0.;
            vref->vector.z = vrefRotComputed;
            break;
        }
    }

    vrefPublisher_.publish (vref);
  }

private:
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle nodeHandlePrivate_;

  int queueSize_;

  ros::Subscriber objectPositionFilteredSubscriber_;
  ros::Subscriber bciCommandSubscriber_;
  ros::Publisher vrefPublisher_;

  double rotationToObjectRef_;
  double distanceToObjectRef_;
  double minDistanceToObjectRef_;

  //variables used in the realignment initial motion
  double kRotInitThreshMaxDeg_;
  double rotInitMaxVel_;
  double kRotInit_;
  double vRefRotInitDeadZone_;

  int commandMode_;

  geometry_msgs::PoseStamped objectPose_;

  //variable used for the circle around motion

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vref");

  try
    {
      Vref application(100);
      if (ros::ok())
	application.spin();
    }
  catch (std::exception& e)
    {
      std::cerr << "fatal error: " << e.what() << std::endl;
      ROS_ERROR_STREAM("fatal error: " << e.what());
      return 1;
    }
  catch (...)
    {
      ROS_ERROR_STREAM("unexpected error");
      return 2;
    }
  return 0;
}

