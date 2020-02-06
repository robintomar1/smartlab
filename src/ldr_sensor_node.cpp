
#include "ros/ros.h"

#include "std_msgs/Int16.h"



class callback
{
public:
	
callback()
{
  /*
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;
	
/**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  
  /*topic which controls the front two row lights*/
	
	two_pub = n.advertise<std_msgs::Int16>("two/light_status", 10);
  	
  /*
   * sensor 7 publishes the ldr sensor value
   * ldr_callback is the function which will be called each time a value is published on 
   * "one/sensor_7_value" topic which reports ldr sensor value
   */
  
  	sensor_7_sub = n.subscribe("one/sensor_7_value", 10, &callback::ldr_Callback,this);
  	
  	
 }
  	void ldr_Callback(const std_msgs::Int16::ConstPtr& msg)
{	
	/*
	 * checking light and projector status
	 */
	ros::param::get("/front_light_status", light_status);
	ros::param::get("/projector_status", projector_status);
	/*
	 *  input : light on projector on
	*  output: projector on and lights off
	*/
	int data = msg->data;
	static int count=0;
  if (data < 700 and data > 600 and projector_status==0)
  {	count++;
  	if (count>5)
  	{
  /*projector has been turned on so lights need to be shut down*/
  	  ros::param::set("/projector_status", 1); 
  	  std_msgs::Int16 light_number;
	  light_number.data=10;
	  two_pub.publish(light_number);
	  light_number.data=20;
	  two_pub.publish(light_number);
	  light_number.data=30;
	  two_pub.publish(light_number);
	  light_number.data=40;
	  two_pub.publish(light_number);
	  light_number.data=60;
	  two_pub.publish(light_number);
	  light_number.data=70;
	  two_pub.publish(light_number);
	  light_number.data=80;
	  two_pub.publish(light_number);
	  ros::param::set("/front_light_status", 0);
	 }
   }
   else 
   {count=0;}
    if (data > 900 and projector_status==1)
  {	
  /*projector has been turned off so lights need to be turned on */
  	  ros::param::set("/projector_status", 0); 
  	  std_msgs::Int16 light_number;
	  light_number.data=11;
	  two_pub.publish(light_number);
	  light_number.data=21;
	  two_pub.publish(light_number);
	  light_number.data=31;
	  two_pub.publish(light_number);
	  light_number.data=41;
	  two_pub.publish(light_number);
	  light_number.data=61;
	  two_pub.publish(light_number);
	  light_number.data=71;
	  two_pub.publish(light_number);
	  light_number.data=81;
	  two_pub.publish(light_number);
	  ros::param::set("/front_light_status", 1);
   }
	 
}

protected :
	ros::Publisher two_pub;
   	ros::Subscriber sensor_7_sub;
	int light_status, projector_status;
	
};







int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

 ros::init(argc, argv, "ldr_sensor_node");
 ros::param::set("/projector_status", 0); 
/*creating objrct of the callback class this object instantiates all the publisher and  
* subscriber needed for our application. we could have done every thing in the main
*  function but we needed to publish from inside the callback function this requirement
*  can not be fulfilled while working the normal way so a object oriented approach 
*  used and a class method is used as a callback function which have acess to other
*  publishers since all belong to the same class they can acess each other easily from
*  inside the function body.
*/
callback callback_1;

 

   ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%
