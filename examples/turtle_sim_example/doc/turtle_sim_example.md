# Turtle Sim Example

This example should demonstrate how the **data flow** can work in a planned state machine and how **flexible** they are even when using them with **Ros**.
The example is base on rafcons [turtle demo](https://rafcon.readthedocs.io/en/latest/tutorials.html#starting-the-basic-turtle-demo-state-machine-using-ros), but the states where modified to use the plugins data flow style, and enriched with PDDL actions.

## Scenario description
The scenario consists of:
- a map with five points (down_left, down_right, top_left, top_right and middle) 
- and three turtles (alice, bob and eve).

![Map](https://rmc-github.robotic.dlr.de/moro/rafcon_task_planner_plugin/blob/develop/examples/turtle_sim_example/doc/turtle_example_scenario.png "Map")



With the following rules:
- turtles can only move between connected points
- if a turtle moves it gets hungry
- if turtles are hungry they can eat eachother

In this example, Alice, Bob and Eve will wander arround in the map, but Bob is an cannibal, therefore he will eat Alice and Eve at some point.



## Requirements 
 - RAFCONː `pip install --user` rafcon orː `git clone git@rmc-github.robotic.dlr.de:common/rafcon.git` (for information about how to install rafcon see: https://github.com/DLR-RM/RAFCON)
 - The Rafcon Task Planner Pluginː `git clone git@rmc-github.robotic.dlr.de:moro/rafcon_task_planner_plugin.git`
 - The Fast Downward Planning Systemː `pip install --user downward-dlr --no-compile` (evt. you have to install the wheel package first) 
 - A running ROS core (as described in rafcons [turtle demo](https://rafcon.readthedocs.io/en/latest/tutorials.html#starting-the-basic-turtle-demo-state-machine-using-ros))
 - A running turtlesim node (also as described in rafcons [turtle demo](https://rafcon.readthedocs.io/en/latest/tutorials.html#starting-the-basic-turtle-demo-state-machine-using-ros))

## Setup
1. Start the ROS core [turtle demo](https://rafcon.readthedocs.io/en/latest/tutorials.html#starting-the-basic-turtle-demo-state-machine-using-ros)
2. Start the turtlesim node [turtle demo](https://rafcon.readthedocs.io/en/latest/tutorials.html#starting-the-basic-turtle-demo-state-machine-using-ros)
3. Start RAFCON with the Plugin in your ros environment
4. Add the following two paths as RAFCON Library Paths:   
   Library: `turtle_lib` Path: `[Repository_PATH]/rafcon_task_planner_plugin/examples/turtle_sim_example/turtle_lib`  
   Library: `turtle_sim_example` Path: `[Repository_PATH]/rafcon_task_planner_plugin/examples/turtle_sim_example`  

If we have a look into the **turtle_sim_example** Library, we can see **turtle_lib** as well as the two state machines **turtle_sim_core_machine** and **turtle_sim_state_machine**.

### trutle_sim_state_machine:  
turtle_sim_state_machine is the one we will execute later. If you open it now, you will see that it just initializes a ros node, and then executing the turtle_sim_core_machine. If you want you can execute it now, but since we didn't plan the core machine yet, nothing spectacular will happen.

### turtle_sim_core_machine:   
This is the machine we will plan, and also the one where everything happens. Inside of the Core Turtles are spawned, moved and eaten.

### turtle_lib:  
This folder contains a bunch of states. These are the bricks used to plan our scenario. Most of them are enriched with an PDDL action, but not all of them so don't be surprised, if the plugin warns you later on, during the planning process. 

## Planning

Now we are ready to plan the core machine. Because this is an example, and not a tutorial everything was prepared for you, and you only have to configure the Task Planner Plugin by hiting **Plan Task**

### Planning
The data we enter in this section is important during the Planning and generation process.
#### State pools
We use our turtle_lib as state pool, it should be:  `[Repository_PATH]/rafcon_task_planner_plugin/examples/turtle_sim_example/turtle_lib`

#### Type file
The type files location is `[Repository_PATH]/rafcon_task_planner_plugin/examples/turtle_sim_example/rtpp-turtle_sim_db.json`  
Since we have only Turtles and Locations, it is really small, and uninteresting.

#### Planner
Use as Planner the **Fast Downward Planning System**.

#### Facts file 
The facts files location is `[repository_PATH]/rafcon_task_planner_plugin/examples/turtle_sim_example/turtle_sim_facts.pddl`  
feel free to play around with it later.

#### State machine name
Since we want to (re)plan the `turtle_sim_core_machine`, we should give our State machine the same name.

#### Save state machine in
We want to replan the turtle_sim_core_machine e.g. overwriting it, therefor we should save our State machine in `[repository_PATH]/rafcon_task_planner_plugin/examples/turtle_sim_example/`

### Runtime
This section is not important during the planning process, but during runtime of the state machine we plan.
#### Runtime Data
If we enter a path to a json file here, the plugin will add a data initialization state as first state of the planned state machine.  
As we know a planner would return something like **move bob middle** and not **move bob x=5, y=5** but ros needs coordinates, and don't know where **middle** is. To circumvent this problem, we use **middle** as an object identifier, and define the Object in a file. its path is `[repository_PATH]/rafcon_task_planner_plugin/examples/turtle_sim_example/turtle_demo_data.json`.
Feel free to have a look into the file, and see all object definitions.

#### Include
At this radio button group we can decide, if we want to add the data directly into the state, or if we only want to reference the data file. This can be usefull in lots of situations, but in our scenario its unimportant. so you can try both if you want. 


## Execution
Now we have planned the core machine, and are ready to watch turtles moving arround. Just open the **turtle_sim_state_machine**, and refresh it bevore execution. 

have fun!


## Last Words on Data Flow

TODO!




