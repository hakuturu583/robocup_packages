# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* Quick summary  
This is ROS packages for ROBOCUP SPL.  
This repogitory is developing by Masaya Kataoka(masaya.kataoka@ams.eng.osaka-u.ac.jp)  
* Version
0.01
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up  
1.install ROS(http://wiki.ros.org/indigo/Installation/Ubuntu)  
2.[create catkin_ws](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)  
3.cd catkin_ws  
4.git clone git@github.com:hakuturu583/robocup_packages.git  
5.catkin_make  
6.roslaunch robocup_launcher nao_navigation.launch

### How to use? ###
* How to run  
  export NAO_IP = 192.168.*.*(setting nao robot ip adress)
  run from bagfiles: roslaunch robocup_launcher play.launch  
  run whole system on local PC and robot: roslaunch robocup_launcher nao_navigation.launch  
  run with gazebo simulator and rviz: roslaunch robocup_launcher nao_virtual.launch  
  run with rviz and moveit(without kinematic simulation): roslaunch robocup_launcher nao_moveit.launch  

# Hardware requirement #
* Stareo Camera
* Nao(Softbank Robotics) is highly recommended

### Who do I talk to? ###

* Repo owner or admin  
Masaya Kataoka(masaya.kataoka@ams.eng.osaka-u.ac.jp,ms.kataoka@gmail.com)  
* Other community or team contact

### Tips ###

if you want to turn off ip notification system of nao
you should turn off notificationreader by modifying /etc/naoqi/autoload.ini and nao restart
