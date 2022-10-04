#include <ros.h>
#include <std_msgs/String.h>

 
 ros::NodeHandle nh;
 
 std_msgs::String str_msg;
 ros::Publisher accel("accel", &str_msg);
 
 float acc = 8.14;
 
 void setup()
 {
   nh.initNode();
   nh.advertise(accel);
 }
 
  void loop()
  {
    char acc_x[5];
    dtostrf(acc, -1, 3, acc_x);
    str_msg.data = acc_x;
    accel.publish( &str_msg );
    nh.spinOnce();
    delay(1000);
  }
