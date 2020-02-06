
#include "ros/ros.h"
#include "ros/time.h"
#include <ros/console.h>
#include "std_msgs/Int16.h"

/*********************************************
 * Callback class
 * 
 * this is the main class of this code contains 
 * all callbacks and ros communication
 *********************************************/

class Callback
{
public:
	
Callback()
{
	ROS_DEBUG(" inside object constructor  ");
  /*
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;
	
   /*
    * these time variables are used to study each sensor data for 5 minutes
    * ros::Time::now() gives the current system time
    */
	begin_1 = ros::Time::now();
	begin_2 = ros::Time::now();
	begin_3 = ros::Time::now();
	begin_4 = ros::Time::now();
	begin_5 = ros::Time::now();
	five_minutes=ros::Duration(300);

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
   
   /*
    * Publishers
    */
	two_pub = n.advertise<std_msgs::Int16>("two/light_status", 10);
  	
  	three_pub = n.advertise<std_msgs::Int16>("three/light_status", 10);
    /*
     * Subscribers
     */	
  	sensor_1_sub = n.subscribe("one/sensor_1_value", 10, &Callback::pir_one_Callback,this);
  	
  	sensor_2_sub = n.subscribe("one/sensor_2_value", 10, &Callback::pir_two_Callback,this);
  	
  	sensor_3_sub = n.subscribe("one/sensor_3_value", 10, &Callback::pir_three_Callback,this);
  	
  	sensor_4_sub = n.subscribe("one/sensor_4_value", 10, &Callback::pir_four_Callback,this);
  	
  	sensor_5_sub = n.subscribe("one/sensor_5_value", 10, &Callback::pir_five_Callback,this);
 }
 
 /*******************************************
  * callback function for first sensor
  *******************************************/
  	void pir_one_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	ROS_DEBUG(" inside one callback ");
	/*
	 * the static variable is used to maintain the sensor data value over 5 minutes
	 */
	static int signal =0;
	ros::Time now = ros::Time::now();
	ROS_DEBUG(" now = %f",now.toSec());
	ROS_DEBUG(" begin_1 = %f",begin_1.toSec());
	ros::Duration time_difference=now-(begin_1+five_minutes);
	ROS_DEBUG(" time plus five min %f",(begin_1+five_minutes).toSec());
	float time_in_sec= time_difference.toSec();
/*
 * this condition will keep adding the sensor data value to a static variable signal for five
 * minutes and if motion is detected even once in those five minutes the condition will 
 * do nothing but if no motion is deected i,e; signal value remains 0 then all lights of 
 * that portion will be switched off
 */	
  ROS_DEBUG(" time_in_sec%f",time_in_sec);
  
  if(time_in_sec<0)
  {
  	signal +=msg->data;
  	ROS_DEBUG(" signal value = %d ",signal);
  }
  else if (time_in_sec>=0)
  {
	  if (! signal)
	  {	ROS_DEBUG(" switching off lights");
	  	  std_msgs::Int16 light_number;
		  light_number.data=130;
		  three_pub.publish(light_number);
		  light_number.data=140;
		  three_pub.publish(light_number);
		  light_number.data=180;
		  three_pub.publish(light_number);
		  light_number.data=190;
		  three_pub.publish(light_number);
	   }
	  	begin_1=ros::Time::now();
	  	signal=0;
  }
 
	 
}
 
 /**********************************************
  * callback function for second sensor
  **********************************************/

void pir_two_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	ROS_DEBUG(" inside one callback ");
	/*
	 * the static variable is used to maintain the sensor data value over 5 minutes
	 */
	static int signal =0;
	ros::Time now = ros::Time::now();
	ROS_DEBUG(" now = %f",now.toSec());
	ROS_DEBUG(" begin_2 = %f",begin_2.toSec());
	ros::Duration time_difference=now-(begin_2+five_minutes);
	ROS_DEBUG(" time plus five min %f",(begin_2+five_minutes).toSec());
	float time_in_sec= time_difference.toSec();
/*
 * this condition will keep adding the sensor data value to a static variable signal for five
 * minutes and if motion is detected even once in those five minutes the condition will 
 * do nothing but if no motion is deected i,e; signal value remains 0 then all lights of 
 * that portion will be switched off
 */	
  ROS_DEBUG(" time_in_sec%f",time_in_sec);
  
  if(time_in_sec<0)
  {
  	signal +=msg->data;
  	ROS_DEBUG(" signal value = %d ",signal);
  }
  else if (time_in_sec>=0)
  {
	  if (! signal)
	  {	ROS_DEBUG(" switching off lights");
	  	  std_msgs::Int16 light_number;
		  light_number.data=110;
		  three_pub.publish(light_number);
		  light_number.data=160;
		  three_pub.publish(light_number);
	   }
	  	begin_2=ros::Time::now();
	  	signal=0;
  }
}
 
 /************************************************
  * callback function for third sensor
  ************************************************/

void pir_three_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	ROS_DEBUG(" inside one callback ");
	/*
	 * the static variable is used to maintain the sensor data value over 5 minutes
	 */
	static int signal =0;
	ros::Time now = ros::Time::now();
	ROS_DEBUG(" now = %f",now.toSec());
	ROS_DEBUG(" begin_3 = %f",begin_3.toSec());
	ros::Duration time_difference=now-(begin_3+five_minutes);
	ROS_DEBUG(" time plus five min %f",(begin_3+five_minutes).toSec());
	float time_in_sec= time_difference.toSec();
/*
 * this condition will keep adding the sensor data value to a static variable signal for five
 * minutes and if motion is detected even once in those five minutes the condition will 
 * do nothing but if no motion is deected i,e; signal value remains 0 then all lights of 
 * that portion will be switched off
 */	
  ROS_DEBUG(" time_in_sec%f",time_in_sec);
  
  if(time_in_sec<0)
  {
  	signal +=msg->data;
  	ROS_DEBUG(" signal value = %d ",signal);
  }
  else if (time_in_sec>=0)
  {
	  if (! signal)
	  {	ROS_DEBUG(" switching off lights");
	  	  std_msgs::Int16 light_number;
		  light_number.data=10;
		  two_pub.publish(light_number);
		  light_number.data=20;
		  two_pub.publish(light_number);
		  light_number.data=60;
		  two_pub.publish(light_number);
		  light_number.data=70;
		  two_pub.publish(light_number);
		  ros::param::set("/front_light_status", 0);
	   }
	  	begin_3=ros::Time::now();
	  	signal=0;
  }

}
/*************************************************
  * callback function for fourth sensor
  *************************************************/

void pir_four_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	ROS_DEBUG(" inside one callback ");
	/*
	 * the static variable is used to maintain the sensor data value over 5 minutes
	 */
	static int signal =0;
	ros::Time now = ros::Time::now();
	ROS_DEBUG(" now = %f",now.toSec());
	ROS_DEBUG(" begin_4 = %f",begin_4.toSec());
	ros::Duration time_difference=now-(begin_4+five_minutes);
	ROS_DEBUG(" time plus five min %f",(begin_4+five_minutes).toSec());
	float time_in_sec= time_difference.toSec();
/*
 * this condition will keep adding the sensor data value to a static variable signal for five
 * minutes and if motion is detected even once in those five minutes the condition will 
 * do nothing but if no motion is deected i,e; signal value remains 0 then all lights of 
 * that portion will be switched off
 */	
  ROS_DEBUG(" time_in_sec%f",time_in_sec);
  
  if(time_in_sec<0)
  {
  	signal +=msg->data;
  	ROS_DEBUG(" signal value = %d ",signal);
  }
  else if (time_in_sec>=0)
  {
	  if (! signal)
	  {	
	  	  ROS_DEBUG(" switching off lights");
	  	  std_msgs::Int16 light_number;
		  light_number.data=30;
		  two_pub.publish(light_number);
		  light_number.data=40;
		  two_pub.publish(light_number);
		  light_number.data=80;
		  two_pub.publish(light_number);
		  ros::param::set("/front_light_status", 0);
	   }
	  begin_4=ros::Time::now();
	  signal=0;
  }

	 
}
 /*************************************************
  * callback function for fifth sensor
  *************************************************/

void pir_five_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	ROS_DEBUG(" inside one callback ");
	/*
	 * the static variable is used to maintain the sensor data value over 5 minutes
	 */
	static int signal =0;
	ros::Time now = ros::Time::now();
	ROS_DEBUG(" now = %f",now.toSec());
	ROS_DEBUG(" begin_5 = %f",begin_5.toSec());
	ros::Duration time_difference=now-(begin_5+five_minutes);
	ROS_DEBUG(" time plus five min %f",(begin_5+five_minutes).toSec());
	float time_in_sec= time_difference.toSec();
/*
 * this condition will keep adding the sensor data value to a static variable signal for five
 * minutes and if motion is detected even once in those five minutes the condition will 
 * do nothing but if no motion is deected i,e; signal value remains 0 then all lights of 
 * that portion will be switched off
 */	
  ROS_DEBUG(" time_in_sec%f",time_in_sec);
  
  if(time_in_sec<0)
  {
  	signal +=msg->data;
  	ROS_DEBUG(" signal value = %d ",signal);
  }
  else if (time_in_sec>=0)
  {
	  if (! signal)
	  {	ROS_DEBUG(" switching off lights");
	  	  std_msgs::Int16 light_number;
		  light_number.data=210;
		  three_pub.publish(light_number);
		  light_number.data=220;
		  three_pub.publish(light_number);
		  light_number.data=230;
		  three_pub.publish(light_number);
		  light_number.data=240;
		  three_pub.publish(light_number);
		  light_number.data=250;
		  three_pub.publish(light_number);
		  light_number.data=260;
		  three_pub.publish(light_number);
		  light_number.data=270;
		  three_pub.publish(light_number);
		  light_number.data=280;
		  three_pub.publish(light_number);
	   }
	  	begin_5=ros::Time::now();
	  	signal=0;
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
   	ros::Time begin_1;
   	ros::Time begin_2;
   	ros::Time begin_3;
   	ros::Time begin_4;
   	ros::Time begin_5;
   	ros::Duration five_minutes;
   	
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

 ros::init(argc, argv, "light_off_node");
/*creating objrct of the callback class this object instantiates all the publisher and  
* subscriber needed for our application. we could have done every thing in the main
*  function but we needed to publish from inside the callback function this requirement
*  can not be fulfilled while working the normal way so a object oriented approach 
*  used and a class method is used as a callback function which have acess to other
*  publishers since all belong to the same class they can acess each other easily from
*  inside the function body.
*/
ROS_DEBUG(" inside main   ");
ROS_DEBUG(" object created   ");
Callback callback_1;


 

   ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%
