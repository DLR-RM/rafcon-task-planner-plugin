# Rafcon Task Planner Plugin

![Rafcon Task Planner Plugin](docs/img/Rafcon_task_planner_plugin.png "The Plugin Configuration")

## Overview

The Rafcon Task Planner Plugin is a [RAFCON](https://github.com/DLR-RM/RAFCON) plugin to interface arbitrary Pddl planner such as the [Fast-Downward planning System](http://www.fast-downward.org/).  
Its purpose is to use such planners to plan a predefined scenario, and generate a Rafcon State machine out of the found plan. 
To achieve this, the plugin extends RAFCON, so that a state's semantic can be expressed via a PDDL Action. Subsequently, a set of such PDDL annotated states can be used to auto-generate a Domain file, find a plan for a handwritten problem and generate a state machine based on the solution.  
One important aspect is that the Task Planner should be able to interface abritrary Planner. Therefore new Planners can be quickly integrated using a python script.

![Pddl Action tab, to annotate a State with an action.](docs/img/rtpp_Action_tab.png "Pddl Annotation")

## Installation
To install the Plugin two steps are required: 
1. `git clone <repository>`
2. Add the Plugin Path to Rafcons PLUGIN_PATH Variable. (see [RAFCON Doc](https://rafcon.readthedocs.io/en/latest/plugins.html)) `RAFCON_PLUGIN_PATH=$RAFCON_PLUGIN_PATH:[repository_path]/source/rafcontpp` 

## Plugin Webpage
Documentation as well as some tutorials are available at the [Plugin's Website](https://rmc-github.robotic.dlr.de/pages/moro/rafcon_task_planner_plugin/) **TODO: CHANGE LINK TO EXTERNAL REPO WEBSITE**
