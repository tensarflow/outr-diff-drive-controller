/*
References:
- Code: https://github.com/merose/diff_drive/blob/ac26b2f526657c8fab7b24a314b48158c92c6045/src/diff_drive/odometry.py
- Notation: https://link.springer.com/book/10.1007/978-3-319-62533-1
*/

#include <main.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <Servo.h>
#include <PID_v1.h>

#define DEBUG 1

// Public variables
int lpwm = 0;             // Final pwm value for left motor
int rpwm = 0;             // Final pwm value for right motor
int len = 250;            // period in ms
int ldir = 1;             // left motor direction
int rdir = 1;             // right motor direction
int lcounter_current = 0; // current left motor tick counter
int rcounter_current = 0; // current right motor tick counter
int lcounter_last = 0;    // left motor tick counter from last iteration
int rcounter_last = 0;    // right motor tick counter from last iteration

ros::NodeHandle nh;
geometry_msgs::TransformStamped tf_home2base; // transformation frame home -> base
tf::TransformBroadcaster broadcaster;         // broadcaster to send transformations
ros::Time current_time = nh.now();
ros::Time last_time = current_time;

double x = 1.0;
double y = 0.0;
double th = 0;
double L = 0.135;                                 // Wheel separation
double DistancePerCount = (TWO_PI * 0.0315) / 20; // 2*PI*R/CPR  WHEEL ENCODER 20 CPR
char base_link[] = "/base_link";
char odomid[] = "/odom";

// generic motion for a period in milisecods
void motion(int lpw, int rpw, int _ldir, int _rdir, int period)
{
    bool llevel_1 = _ldir;
    bool llevel_2 = !(_ldir);
    bool rlevel_1 = _rdir;
    bool rlevel_2 = !(_rdir);

    // Set tick counter increment variables for encoder callback
    if (llevel_1 == HIGH && llevel_2 == LOW)
        ldir = 1;
    if (llevel_1 == LOW && llevel_2 == HIGH)
        ldir = -1;
    if (rlevel_1 == HIGH && rlevel_2 == LOW)
        rdir = 1;
    if (rlevel_1 == LOW && rlevel_2 == HIGH)
        rdir = -1;

    // Set speed
    analogWrite(D1, lpw);
    analogWrite(D6, rpw);

    // Give direction
    digitalWrite(D2, llevel_1);
    digitalWrite(D3, llevel_2);
    digitalWrite(D4, rlevel_1);
    digitalWrite(D5, rlevel_2);
}

//  All subscriber messages callbacks here
void leftCallback(const std_msgs::Int16 &msg)
{
    lpwm = abs(msg.data);
    rpwm = abs(msg.data);
    motion(lpwm, rpwm, LOW, HIGH, len);
    Serial.println("Driving left.");
}
void rightCallback(const std_msgs::Int16 &msg)
{
    lpwm = abs(msg.data);
    rpwm = abs(msg.data);
    motion(lpwm, rpwm, HIGH, LOW, len);
    Serial.println("Driving right.");
}
void forwardCallback(const std_msgs::Int16 &msg)
{
    lpwm = abs(msg.data);
    rpwm = abs(msg.data);
    motion(lpwm, rpwm, HIGH, HIGH, len);
    Serial.println("Driving forward.");
}
void backwardCallback(const std_msgs::Int16 &msg)
{
    lpwm = abs(msg.data);
    rpwm = abs(msg.data);
    motion(lpwm, rpwm, LOW, LOW, len);
    Serial.println("Driving backwards.");
}

ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);

// GPIO ISRs (Interrupt service routines) for encoder changes
void ICACHE_RAM_ATTR lencode()
{
    lcounter_current = lcounter_current + ldir;
    Serial.print("Left encoder counting: ");
    Serial.println(lcounter_current);
}

void ICACHE_RAM_ATTR rencode()
{
    rcounter_current = rcounter_current + rdir;
    Serial.print("Right encoder counting: ");
    Serial.println(rcounter_current);
}

void setup()
{
    // configure GPIO's
    pinMode(D1, OUTPUT); // ENA, PWM left motor (A)
    pinMode(D2, OUTPUT); // IN1
    pinMode(D3, OUTPUT); // IN2
    pinMode(D4, OUTPUT); // IN3
    pinMode(D5, OUTPUT); // IN4
    pinMode(D6, OUTPUT); // ENB, PWM right motor (B)
    pinMode(D7, INPUT);  // Left encoder
    pinMode(D8, INPUT);  // Right encoder

    Serial.begin(115200);
    setupWiFi();
    delay(2000);

    // configure interrupts to their ISR's
    attachInterrupt(D7, lencode, RISING); // Setup right encoder interrupt
    attachInterrupt(D8, rencode, RISING); // Setup left encoder interrupt
    sei();                                // Enable interrupts

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    broadcaster.init(nh);
    nh.subscribe(sub_r);
    nh.subscribe(sub_l);
    nh.subscribe(sub_f);
    nh.subscribe(sub_b);
}

void loop()
{
    if (nh.connected())
    {
        current_time = nh.now();

        // Calculate new position
        double dt = current_time.toSec() - last_time.toSec();
        double d_l = (lcounter_current - lcounter_last) * DistancePerCount; // delta left wheel linear distance (s: strecke)
        double d_r = (rcounter_current - rcounter_last) * DistancePerCount; // delta right wheel linear distance
        double d_c = (d_l + d_r) / 2;                                       // delta center linear distance
        double d_th = (d_r - d_l) / L;                                      // delta angle
        double d_x;
        double d_y;

        if (d_l == d_r)
        {
            d_x = d_l * cos(th);
            d_y = d_l * sin(th);
        }
        else
        {
            double r_c = d_c / d_th;                                             // radius to instantaneous center of curvature (ICC)
            double icc_x = x - r_c * sin(th);                                    // x of  ICC
            double icc_y = y - r_c * sin(th);                                    // x of  ICC
            d_x = cos(d_th) * (x - icc_x) - sin(d_th) * (y - icc_y) + icc_x - x; // total distance travelled in x
            d_y = sin(d_th) * (x - icc_x) + cos(d_th) * (y - icc_y) + icc_y - y; // total distance travelled in y
        }

        x += d_x;
        y += d_y;
        th = fmod((th + d_th), TWO_PI);

        // Update velocity
        double v_x;
        double v_y = 0; // Differential drive kinematics has no y-velocity. Think about it :)
        double v_th;

        Serial.print("dt: ");
        Serial.println(dt);

        if (dt > 0)
        {
            v_x = d_c / dt;
            v_th = d_th / dt;
        }
        else
        {
            v_x = 0;
            v_th = 0;
        }

        // Update for next iteration
        lcounter_last = lcounter_current;
        rcounter_last = rcounter_current;
        last_time = current_time;

        tf_home2base.header.frame_id = odomid;
        tf_home2base.child_frame_id = base_link;
        tf_home2base.transform.translation.x = x;
        tf_home2base.transform.translation.y = y;
        tf_home2base.transform.rotation = tf::createQuaternionFromYaw(th);
        tf_home2base.header.stamp = current_time;
        broadcaster.sendTransform(tf_home2base);
    }

    nh.spinOnce();
    delay(800);
}