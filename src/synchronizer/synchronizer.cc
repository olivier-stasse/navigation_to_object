#include <stdexcept>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
      objectPositionFilteredPublisher_ ()
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
    bool isMessageOk = true;

    //FIXME: do something smart here.

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
