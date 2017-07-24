/*
   This project is just for test slamware, and give an simple tutorial to use slamware SDK on ROS.

if you have any question, can contact with kint.zhao E-MAIL: huasheng_zyh@163.com.

More information from following:
slamtec:      http://www.slamtec.com/en/
slamware SDK: http://www.slamtec.com/cn/Slamware
slamware API: https://wiki.slamtec.com/pages/viewpage.action?pageId=1016252
*/

Do some application using slamware SDK on ubuntu.
1. create an example by cmake.	notice: the order of static libs.
2. create an ros package to connect slamware robot platform by tcp/ip. 
3. finish the slamwareNode 


NOTICE:
 1) you should run the slamcore on your robot, mark the IP_addres. and make sure slamCore run OK!
 2) modify the launch/view_slamwareAPP.launch for your robot system.
 3) upgrade your slamware SDK from http://www.slamtec.com/cn/Slamware
 4) run : roslaunch slamwareAPP view_slamwareNode.launch



