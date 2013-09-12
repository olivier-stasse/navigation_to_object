#include <stdexcept>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/console.h>

class Synchronizer
{
public:
  typedef message_filters::sync_policies::ApproximateTime
  <sensor_msgs::Imu,
   geometry_msgs::WrenchStamped,
   geometry_msgs::WrenchStamped,
   geometry_msgs::PoseStamped> syncPolicy_t;


  explicit Synchronizer (int queueSize)
    : nodeHandle_ (""),
      nodeHandlePrivate_ ("~"),
      queueSize_ (queueSize),
      imuSubscriber_ (nodeHandle_, "imu", queueSize_),
      forceLeftSubscriber_ (nodeHandle_, "force_left_foot", queueSize_),
      forceRightSubscriber_ (nodeHandle_, "force_right_foot", queueSize_),
      objectPositionSubscriber_ (nodeHandle_, "object_position", queueSize_),
      synchronizer_ (syncPolicy_t(queueSize_)),
      objectPositionFilteredPublisher_ (),
      floorLeftFoot_ (true),
      floorRightFoot_ (true),
      footInAirThreshold_ (100.0),
      footOnFloorThreshold_ (400.0)
    
  {
    // ApproximateTime takes a queue size as its constructor argument,
    // hence syncPolicy_t(10)

    synchronizer_.connectInput
      (imuSubscriber_,
       forceLeftSubscriber_,
       forceRightSubscriber_,
       objectPositionSubscriber_);

    synchronizer_.registerCallback
      (boost::bind(&Synchronizer::callback, this, _1, _2, _3, _4));

    objectPositionFilteredPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>
      ("object_position_filtered", queueSize_);
  }

  ~Synchronizer ()
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

  void callback (const sensor_msgs::ImuConstPtr& imu,
		 const geometry_msgs::WrenchStampedConstPtr& forceLeftFoot,
		 const geometry_msgs::WrenchStampedConstPtr& forceRightFoot,
		 const geometry_msgs::PoseStampedConstPtr& objectPosition)
  {
    bool isMessageOk = false;

    //publish the message if the left foot is leaving the floor
    if(floorLeftFoot_ == true) //left foot in contact with the floor
    {
        if(forceLeftFoot->wrench.force.z < footInAirThreshold_)
        {
            floorLeftFoot_ = false; //the foot is in the air
            isMessageOk = true;
        }
        else
        {
        }
    }
    else
    {
        if(forceLeftFoot->wrench.force.z > footOnFloorThreshold_)
        {
            floorLeftFoot_ = true; //the foot is on the floor
        }
        else
        {
        }
    }

    //publish the message if the right foot is leaving the floor
    if(floorRightFoot_ == true) //right foot in contact with the floor
    {
        if(forceRightFoot->wrench.force.z < footInAirThreshold_)
        {
            floorRightFoot_ = false; //the foot is in the air
            isMessageOk = true;
        }
        else
        {
        }
    }
    else
    {
        if(forceRightFoot->wrench.force.z > footOnFloorThreshold_)
        {
            floorRightFoot_ = true; //the foot is on the floor
        }
        else
        {
        }
    }

    //publish the message when one foot leave the floor
    if (isMessageOk)
      objectPositionFilteredPublisher_.publish (objectPosition);
  }

private:
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle nodeHandlePrivate_;

  int queueSize_;

  message_filters::Subscriber<sensor_msgs::Imu> imuSubscriber_;
  message_filters::Subscriber<geometry_msgs::WrenchStamped> forceLeftSubscriber_;
  message_filters::Subscriber<geometry_msgs::WrenchStamped> forceRightSubscriber_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> objectPositionSubscriber_;
  message_filters::Synchronizer<syncPolicy_t> synchronizer_;

  ros::Publisher objectPositionFilteredPublisher_;

  bool floorLeftFoot_; //true if the left foot is in contact with the floor
  bool floorRightFoot_; //true if the right foot is in contact with the floor

  double footInAirThreshold_; //below this value we consider the foot in the air
  double footOnFloorThreshold_; //above this value we consider in contact with the floor

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "synchronizer");

  try
    {
      Synchronizer synchronizer(100);
      if (ros::ok())
	synchronizer.spin();
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
