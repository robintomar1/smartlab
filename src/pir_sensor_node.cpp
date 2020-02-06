
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
	two_pub = n.advertise<std_msgs::Int16>("two/light_status", 10);
  	
  	three_pub = n.advertise<std_msgs::Int16>("three/light_status", 10);
  	
  	sensor_1_sub = n.subscribe("one/sensor_1_value", 10, &callback::pir_one_Callback,this);
  	
  	sensor_2_sub = n.subscribe("one/sensor_2_value", 10, &callback::pir_two_Callback,this);
  	
  	sensor_3_sub = n.subscribe("one/sensor_3_value", 10, &callback::pir_three_Callback,this);
  	
  	sensor_4_sub = n.subscribe("one/sensor_4_value", 10, &callback::pir_four_Callback,this);
  	
  	sensor_5_sub = n.subscribe("one/sensor_5_value", 10, &callback::pir_five_Callback,this);
 }
  	void pir_one_Callback(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data ==1)
  {
  	  std_msgs::Int16 light_number;
	  light_number.data=131;
	  three_pub.publish(light_number);
	  light_number.data=141;
	  three_pub.publish(light_number);
	  light_number.data=181;
	  three_pub.publish(light_number);
	  light_number.data=191;
	  three_pub.publish(light_number);
   }
	 
}

void pir_two_Callback(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data ==1)
  {
  	  std_msgs::Int16 light_number;
	  light_number.data=111;
	  three_pub.publish(light_number);
	  light_number.data=151;
	  three_pub.publish(light_number);
	  light_number.data=161;
	  three_pub.publish(light_number);
	  light_number.data=171;
	  three_pub.publish(light_number);
	  
   }
	
}

void pir_three_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	int projector_status;
	ros::param::get("/projector_status", projector_status);
  if (msg->data==1 and projector_status==0)
  {
  	  std_msgs::Int16 light_number;
	  light_number.data=11;
	  two_pub.publish(light_number);
	  light_number.data=21;
	  two_pub.publish(light_number);
	  light_number.data=61;
	  two_pub.publish(light_number);
	  light_number.data=71;
	  two_pub.publish(light_number);
	  ros::param::set("/front_light_status", 1);
   }
	
}

void pir_four_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	int projector_status;
	ros::param::get("/projector_status", projector_status);
  if (msg->data==1 and projector_status==0)
  {
  	  std_msgs::Int16 light_number;
	  light_number.data=31;
	  two_pub.publish(light_number);
	  light_number.data=41;
	  two_pub.publish(light_number);
	  light_number.data=81;
	  two_pub.publish(light_number);
	  ros::param::set("/front_light_status", 1);
   }
}
void pir_five_Callback(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data ==1)
  {
  	  std_msgs::Int16 light_number;
	  light_number.data=211;
	  three_pub.publish(light_number);
	  light_number.data=221;
	  three_pub.publish(light_number);
	  light_number.data=231;
	  three_pub.publish(light_number);
	  light_number.data=241;
	  three_pub.publish(light_number);
	  light_number.data=251;
	  three_pub.publish(light_number);
	  light_number.data=261;
	  three_pub.publish(light_number);
	  light_number.data=271;
	  three_pub.publish(light_number);
	  light_number.data=281;
	  three_pub.publish(light_number);
   }
	 
}
protected :
	ros::Publisher two_pub;
	ros::Publisher three_pub;
   	ros::Subscriber sensor_1_sub;
   	ros::Subscriber sensor_2_sub;
   	ros::Subscriber sensor_3_sub;
   	ros::Subscriber sensor_4_sub;
   	ros::Subscriber sensor_5_sub;
   	
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

 ros::init(argc, argv, "pir_sensor_node");
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
