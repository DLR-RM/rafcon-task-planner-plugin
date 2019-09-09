---
# You don't need to edit this file, it's empty on purpose.
# Edit theme's home layout instead if you wanna make some changes
# See: https://jekyllrb.com/docs/themes/#overriding-theme-defaults
layout: default
title: Home
---
# Rafcon Task Planner Plugin
![The Rafcon Task Planner Plugin](assets/images/RafconTPPMain.png "An Overview of the Rafcon Task Planner Plugin")


The Task Planner is a plugin for [RAFCON](https://github.com/DLR-RM/RAFCON/), which allows to automatically create, or extend state machines for given tasks.  
To achieve this, states in RAFCON are annotated with semantic information, using the language [PDDL](https://en.wikipedia.org/wiki/Planning_Domain_Definition_Language). Then a semantic planner such as [The Fast-Downward Planning System](http://www.fast-downward.org/) can use this information, to find a solution for a given task. Afterwards the Plugin processes this solution, in order to create a state machine, or extend an existing one. 


## Features
+ [Annotate States with PDDL Actions](pages/documentation/PDDLActionTab.md)
+ [Automatically generate a State machine for a given Task](pages/tutorials/restaurant_tutorial.md)
+ [Automatically extend a State machine](pages/tutorials/turtle_sim_example.md)
+ [Configure the Plugin to inject runtime Data initialization into the state machine](pages/tutorials/turtle_sim_example.md)
+ [Easily integrate a Semantic Planner](pages/documentation/PlannerIntegration.md)


## GitHub
Here is the GitHub repository of the [RAFCON task planner plugin](https://github.com/DLR-RM/rafcon-task-planner-plugin).


