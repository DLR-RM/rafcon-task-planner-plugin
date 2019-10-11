# RAFCON Task Planner Plugin

![RAFCON Task Planner Plugin](docs/img/Rafcon_task_planner_plugin.png "The Plugin Configuration")

## Overview
The RAFCON Task Planner Plugin is a [RAFCON](https://github.com/DLR-RM/RAFCON) plugin to interface arbitrary Pddl planner such as the [Fast-Downward planning System](http://www.fast-downward.org/).  
It's purpose is to use such planners to plan a predefined scenario, and generate a RAFCON state machine out of the found plan. 
To achieve this, the plugin extends RAFCON, so that a state's semantic can be expressed via a PDDL Action. Subsequently, a set of such PDDL annotated states can be used to auto-generate a Domain file, find a plan for a handwritten problem and generate a state machine based on the solution.  
One important aspect is that the Task Planner Plugin is able to interface abritrary Planner. So new Planners can be quickly integrated using a python script. This mechanism was for example used to successfully integrate the [Fast Downward](http://www.fast-downward.org/), as well as the [Fast-Forward](https://fai.cs.uni-saarland.de/hoffmann/ff.html) Planning System.

![Pddl Action tab, to annotate a State with an action.](docs/img/rtpp_Action_tab.png "Pddl Annotation")

## Installation
Currently the plugin is available on GitHub and PyPi. Information about how to obtain and install it from one of these platforms, is available at the homepage's [Getting Started Section](https://dlr-rm.github.io/rafcon-task-planner-plugin/pages/documentation/GettingStarted.html). 

## Plugin Webpage
Documentation as well as some tutorials are available at the [Plugin's Website](https://dlr-rm.github.io/rafcon-task-planner-plugin/)

