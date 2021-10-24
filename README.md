Guide to Introduction to Robotics
==========

If you are as lost as I was when I entered this class follow this guide

Getting Started
===========

1. Install the ros distribution that you want (preferebly [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) , with desktop-full, if you couldn't install it go with [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu))

2. Navigate to Home folder using:

            cd

3. Clone this repository (usually to clone a repository you do: git clone git_link):

            git clone https://github.com/socrob/autonomous_systems.git
            
Whenever the repo is updated you can get the latest updates with:
        
            cd $HOME/autonomous_system
            git pull origin master
            
            
Creating a workspace and installing most of what you will need
===================

1. Create a folder for your ros workspace in your system:

            mkdir -p ~/catkin_ws

3.Clone this git!!:

            git clone https://github.com/tometaro07/src.git

2. Install python catkin tools (to be able to compile with the new catkin build system, instead of catkin_make which is the old one)

            sudo apt-get install python-catkin-tools

3. Source (enable) ROS on your system (only one time, this line should not be there on your .bashrc as it will be bashed by the scripts structure)

            source /opt/ros/kinetic/setup.bash
            
4. Download the following packages:

            sudo apt-get install ros-kinetic-tf2-*
            sudo apt-get install ros-kinetic-hls-lfcd-lds-driver
            sudo apt-get install ros-kinetic-navigation
            sudo apt-get install ros-kinetic-robot-localization             #only if using melodic
            
            
Helpful commands
================

1. Whenever you install a package in source, go to workspace and use:
            
            rosdep check --from-path src --ignore-src
            
if it says "All system dependencies have been satisfied" you are good if not use:

            rosdep install --from-path src --ignore-src
            
and it will install any dependencies that your package will need;

2. When you want to unistall a package that you installed through "sudo apt-get" use:

            sudo apt --purge remove package_name #for example for navigation package you would use ros-kinetic-navigation
            
followed by:

            sudo apt --purge autoremove ros-kinetic-mavros

which will remove everything that was installed to use this package, if it is not being used by another package;

3. When you remove a package from your source use on workspace:

            catkin clean
            catkin_make
            
this will remove all your build and devel from workspace and then build it again without the things from the unwanted package.

You can also use:

            catkin clean PKGNAME
            catkin clean --dependents PKGNAME
            
to clean a single package, but I'm a bit skeptic so I prefer the other way;
