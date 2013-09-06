#include <stdexcept>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

class Vref
{
public:
  explicit Vref (int queueSize)
    : nodeHandle_ (""),
      nodeHandlePrivate_ ("~"),
      queueSize_ (queueSize),
      objectPositionFilteredSubscriber_ (),
      vrefPublisher_ ()
  {
    typedef boost::function<void (const geometry_msgs::PoseStampedConstPtr&)>
      callback_t;

    callback_t callback =
      boost::bind (&Vref::callback, this, _1);

    objectPositionFilteredSubscriber_ =
      nodeHandle_.subscribe
      ("object_position_filtered", queueSize_,
       callback);

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

  void callback (const geometry_msgs::PoseStampedConstPtr& objectPosition)
  {
    geometry_msgs::Vector3StampedPtr vref =
      boost::make_shared<geometry_msgs::Vector3Stamped> ();

    vref->header = objectPosition->header;
    vref->vector.x = 0.1;
    vref->vector.y = 0.;
    vref->vector.z = 0.;

    vrefPublisher_.publish (vref);
  }

private:
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle nodeHandlePrivate_;

  int queueSize_;

  ros::Subscriber objectPositionFilteredSubscriber_;
  ros::Publisher vrefPublisher_;
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

