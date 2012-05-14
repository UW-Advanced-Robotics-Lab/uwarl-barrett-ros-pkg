barrett-ros-pkg -- README
Barrett Technology Inc.
2012-05-11

Barrett Technology's ROS repository is an abstraction of Libbarrett.  Libbarrett
is a real-time controls library written in C++ that runs Barrett Technology's 
products, including the WAM Arm and the BH8-280 BarrettHand.

Our ROS repository requires an installed version of Libbarrett.  To check out 
the latest version of Libbarrett:

svn http://web.barrett.com/svn/libbarrett/tags/libbarrett-1.0.0

Please view the README.txt file within Libbarrett for installation instructions.

The wam_robot stack is designed to be run on a WAM PC-104 or external control 
PC.  

The wam_common stack is designed as the interface to communicate with the 
functionality exposed by the wam_node.

The wam_sim stack is in development and should be released shortly.

Please contact us with any functionality or feature requests.  We appreciate 
contributions and collaborations.

Libbarrett is free software: you can redistribute it and/or
modify it under the terms of the GNU General Public License as published by the
Free Software Foundation.

Contact us at:
support@barrett.com
http://www.barrett.com/
+1-617-252-9000

Barrett Technology
625 Mount Auburn Street
Cambridge, MA 02138
USA
