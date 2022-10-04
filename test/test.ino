// Basic demo for accelerometer/gyro readings from Adafruit ISM330DHCX

#include <Adafruit_ISM330DHCX.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher acc("acc", &str_msg);

Adafruit_ISM330DHCX ism330dhcx;
void setup(void) {
  nh.initNode();
  nh.advertise(acc);

  Serial.begin(115200);
 
  if (!ism330dhcx.begin_I2C()) {
    // if (!ism330dhcx.begin_SPI(LSM_CS)) {
    // if (!ism330dhcx.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  
  char acc_x[5];
  dtostrf(accel.acceleration.x, 6, 2, acc_x);
  str_msg.data = acc_x;
  acc.publish(&str_msg);
  nh.spinOnce();
  delay(100);

}
