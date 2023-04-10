# AlphaBot2-Pi with ROS2 in Python
The code has been developed for my final project of the Automation and Robotics course in Politechnika Rzeszowska.
This repo contains ROS2 packages for the Waveshare AlphaBot2-Pi mobile robot:
  * alphabot2: nodes used to implement certain functionalities of AlphaBot2
  * alphabot2_interfaces: custom interfaces (ROS messages)

The current version includes nodes that:
  * manage the motors 
  * manage the IR obstacle sensors 
  * implement camera QR code detection
  * implement real-time visual drawing of the robots path
  * implement obstacle avoiding program

ROS2 nodes topology:

  Obstacle avoiding : 
  ![Screenshot](https://github.com/mykasero/Alfabot2/blob/e6c31d9744f627585acdf0143f8467f84ec04e3c/Nodes_Avoiding.png)
  
  
  QR markers searching: 
  
  
  
  ![Screenshot](https://github.com/mykasero/Alfabot2/blob/e6c31d9744f627585acdf0143f8467f84ec04e3c/Nodes_QR.png)
