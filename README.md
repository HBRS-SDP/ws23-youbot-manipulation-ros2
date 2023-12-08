# ws23-youbot-manipulation-ros2
<p align="center">
  <img src="http://www.youbot-store.com/wiki/skins/youbot/images/youbot.jpg" alt="YouBot Image"/>
</p>

## ROS2 based youBot-arm manipulation using C++
Developing codebase in C++ for implementing these tasks:
1. Move the arm to given joint angles using youBot driver interface.
2. Move the arm to a given pose by performing Inverse Kinematics using KDL library.
3. Move the arm to a given pose by velocity control.
4. Move the arm to a given pose using the Popov-Vereshchagin Hybrid Dynamics solver 
interface from KDL.


### Technologies
- Robot Platform: [KUKA youBot](http://www.youbot-store.com/developers/kuka-youbot-kinematics-dynamics-and-3d-model-81)
- OS: Ubuntu 20.04 / 22.04 LTS
- Middleware:ROS2 Rolling
- Libraries: youbot_driver, KDL


### Setup and Usage of customized Package
Please refer the detailed steps provided to setup the environment and use it for the project in the Wiki page - [here](https://github.com/HBRS-SDP/ws23-youbot-manipulation-ros2/wiki)
