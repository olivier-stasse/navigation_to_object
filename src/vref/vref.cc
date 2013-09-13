#include <stdexcept>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Int8.h>

#include <math.h>

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
      minDistanceToObjectRef_(0.4),//m param
      kRotInitThreshMaxDeg_(15.0),//degree param
      rotInitMaxVel_(0.2),//rad/s param
      kRotInit_(0.0),//1/s
      vRefRotInitDeadZone_(5.0),//deg param
      commandMode_(0)
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

  void computeRotationToObjectRef(const geometry_msgs::PoseStampedConstPtr& objectPosition, double& rotationToObjectRef)
  {
    rotationToObjectRef = atan2( objectPosition->pose.position.y, objectPosition->pose.position.x);
  }

  void computeDistanceToObjectRef(const geometry_msgs::PoseStampedConstPtr& objectPosition, double& distanceToObjectRef)
  {
    distanceToObjectRef = sqrt(objectPosition->pose.position.y*objectPosition->pose.position.y+\
                               objectPosition->pose.position.x*objectPosition->pose.position.x);
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
    computeRotationToObjectRef( objectPosition, rotationToObjectRef_);
    computeDistanceToObjectRef( objectPosition, distanceToObjectRef_);

// check the control mode set by the callback bci _command


//copy and send the velocity command
    geometry_msgs::Vector3StampedPtr vref =
      boost::make_shared<geometry_msgs::Vector3Stamped> ();

    vref->header = objectPosition->header;
    vref->vector.x = 0.1;
    vref->vector.y = 0.;
    vref->vector.z = 0.;

    vrefPublisher_.publish (vref);
  }

  void callbackBciCommand(const std_msgs::Int8ConstPtr& bciCommand)
  {
     //update the mode of control
     //commandMode:
     //0 idle mode
     //1 go forward
     //2 turn right
     //4 turn left
     //5 start assistive control
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

