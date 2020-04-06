---
layout: default
title: Robex - Seismic Network Example
youtubeId: -wXQf0b1bqQ
---

# 3. Robex - Seismic Network Example

This example is a **showcase**, and demonstrates how the plugin can be used to automatically generate a state machine, that is executable on a **ROS-Gazebo stack**.<br>
It is based on a moon analogue mission, conducted at Mt. Etna by the ROBEX Alliance in the years 2016 and 2017. During the ["Autonomous Passive Seismic Experiment"](http://www.robex-allianz.de/en/about-robex/demo-missions/), a rover called the "LRM" had to take four seismometers (RU) from a Lander, and place them 
as sensor network on the ground. To ensure that each unit functioned correctly, the rover had to drive to the deploy location, level the ground, optimizie the contact between sensor and surface, and test it by giving a ground impulse. 
To achieve autonomy, RAFCON was used and a state machine was manually prepared in advance. <br>
<br>
This example shows, how to go on step further by also generating the state machine, needed to accomplish the task automatically. To simulate the environment and the LRM, Gazebo and ROS where used. 
<!--https://www.hjkc.de/_blog/2017/07/05/8319-raumfahrt-mission-robex-unter-mondbedingungen-auf-dem-vulkan-aetna-durchgefuehrt/-->

- [3.1 Scenario Description](#31-scenario-description)
- [3.2 LRM Skills](#32-lrm-skills)
- [3.3 Showcase Video](#33-showcase-video)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>




![Robex Seismic Network Scenario Overview](../../assets/images/tutorials/lrm/ROBEX_LRM.png "Scenario Overview")
## 3.1 Scenario Description



Initially all four Remote Units (RUs), which is a more abstract name for the seismometers, are attached to the lander, and the LRM is located next to it.<br>
LRM's task is, to arrange the RUs, to form a sensor network. Therefore it has to graps each RU individually from the lander, carry it to it's position in the network, and deploy it.<br>
The LRM deploys a RU, by first grasping the seismometer from it's back, and use it afterwards to level the ground, before placing the RU, and optimizing it's ground contact. The last step of deployment is to test the sensors functionyllity. So the LRM uses it's manipulator, to give a ground impulse.


## 3.2 LRM Skills
The LRM has some capabilities, which can be used by the plugin to find a plan, and generate the state machine, necessary to fullfill the task. These Skills are listed below:  

<img src="../../assets/images/tutorials/restaurant/restaurant_tutorial_overview.jpg"  alt="Scenario Overview" style="display:block; margin-left: auto; margin-right: auto; width:50%;">
<table>
<tr valign="top"><th>Skill</th><th>Description</th></tr>
<tr><td>Analyse Ground</td><td>Detects, wether the surface is suitable for placing a remote unit.</td></tr>
<tr><td>Carry RU</td><td>Carries a RU to it's defined deploy location.</td></tr>
<tr><td>Complete Deployment</td><td>A PDDL related helper "skill" for convenience.</td></tr>
<tr><td>Grasp From Lander</td><td>Grasps a remote unit from the lander.</td></tr>
<tr><td>Ground Impulse</td><td>Gives an ground impulse with it's manipulator.</td></tr>
<tr><td>Level Ground</td><td>Uses an attached remote unit to level the ground.</td></tr>
<tr><td>Locate World Objects</td><td>Updates the world model of the rover.</td></tr>
<tr><td>Navigate to RU on Lander</td><td>Navigates to a position nearby the lander, where it can grasp a particual RU from.</td></tr>
<tr><td>Optimize Contact</td><td>Optimizes the contact between a remote unit and the ground.</td></tr>
<tr><td>Place RU</td><td>Attaches it's manipulator to a RU, it carries, and takes it from it's back.</td></tr>
<tr><td>Release RU</td><td>Release a RU, it's manipulator is attached to.</td></tr>
<tr><td>Retract From RU</td><td>Separates itself from a remote unit, to gain full navigation capabilities again.</td></tr>
</table>

## 3.3 Showcase Video
{% include youtubePlayer.html id=page.youtubeId %}