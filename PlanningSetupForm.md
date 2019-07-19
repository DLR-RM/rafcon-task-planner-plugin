
# The Planning Setup Form

INSTERT PICTURE

The Planning setup form can be opened by clicking the "Plan Task" Button in the Menu bar of [RAFCON](https://dlr-rm.github.io/RAFCON/).  
It's the main Window of the Plugin, and the part where to configure a new Task, or get information about the current configuration.

## Planning Section

The Planning section contains all fields relevant for Planning and the State machine generation process. This section explains the fields in Detail.

### State pools

A state pool is a directory, containing PDDL-annotated States (but not all States in the directory have to be annotated).  
In the State pools field you can choose state pools you want allow the plugin to use in order to solve a particular Task.
During the generation process, all State pools are added as Rafcon Libraries, in which the directory name is used as Library name.    
The state pools field consists of two fields: A Directory chooser and a text field.  
All Directories choosen, are added to the text field as ':'- separated list. If you want to remove a State pool from the Task, just remove it from the list in text field. If you have a list already, you can easily paste it into the text field.

**Example**
```
/home/turtle_lib: tree
── turtle_lib
│   ├── clear_field
│   ├── __init__.py
│   ├── init_ros_node
│   ├── move_to_position
│   ├── teleport_turtle
│   └── turtle_position_subscriber

In this case for example 'move_to_position' is annotated with PDDL, and 'init_ros_node' is not 
(that's not recognizable in this view). The Statepool would be '/home/turtle_lib'.
Now adding '/home/turtle_lib' to the text field, would enable the Plugin to use 'move_to_position', 
as well as 'teleport_turtle' (which are annotated, that's not recognizable in this view) for planning. 
You will receive a warining for 'clear_field', 'init_ros_node' and 'turtle_position_subscriber'
(which are not annotated, that's also not recognizable in this view), but that dosen't matter as long as 
its intended.
The added Rafcon library would look like: 
                         Library: trutle_lib 
                         Path: /home/trutle_lib 
```

### Type File

Since PDDL uses Types and heredity, the plugin needs information about the Type hierarchy. This should be provided here.
The Type File Field is a File chooser. It expects a File of type json. It should contain the Type hierarchy as json dict, containing child types as keys and parent types as values. One child can only have one parent, and the defined hierarchy has to have exactly one root type. If you don't have a root type, you can add a hierarchy level, and derive your roots from 'Object'.
The root type (obviously) has no parent.  
**Important:** All your types used in the annotated States, or in your facts file have to be part of this hierarchy.

**Example**
```
file: my_types.json:
{
"Location":"Object",
"City":"Location",
"Country":"Location"
}
In this file, 'Location' is derived from 'Object', and the types 'City' and 'Country' are 
derived from Location. 'Object' is the root type.
```
### Planner

In this field it's settled which built-in planner should be used to Plan a particular Task, or a separate planner script should be used. If a built-in planner is not correctly installed, a notification is shown.  
The Planner field is a Drop-down entry Chooser, where you can select one Planner. If you choose `Other...`, you indicate that you will provide a planner script in the 'Planner script Location' field, and use your own planner.  
**Important:** If you forget to choose `Other...` your planner script will not be taken into consideration, but the choosen Planner will be used instead.

**Example**

Field Value | Description
---|-----
Fast Downward Planning System | The Fast Downward Planning System is used for Planning.
Fast Downward Planning System (!) Unavailable | The Fast Downward System is not (correctly) installed.
Other... | A Planner script provided in the 'Planner script Location' field will be usd.

### Planner Script Location

This field expects a python script, which will be responsible for the planning part of a Task. How to implement such a script can be found in this [Section](ToDO: provide LINK).  
The Planner Script Location field is a file chooser. The file choosen here will be used for Planning.  
**Important:** If `Other...` is not selected in the 'Planner' field, this field will be stored, but ignored during Task process.

**Example**

```
Field value: /my/scripts/my_planner_script.py

If Planner Scrip Location contains the value above, and 'Other...' is selected in the Planner field, 
the script 'my_planner_script.py' will be used for Planning in the Task.

```

### Planner Argv

If you want to configure the Planner e.g. The Fast Downward Planning system, you can insert an argument vector here. 
This field is an text entry, and it behaves like entering arugment into the console, e.g. Values enterd into this field are given to the planner script as space separated array. So all built in planners can be configured as specified in the original Planner documentation.

**Example**

```
When using the Fast Downward Planning System:
Possible planner Argv field value: --search "astar(lmcut())"
```

### Facts File

This Filechooser expects a Facts File written in Pddl. A Planning Task usually consits of a facts, and a domain file. Since the Plugin is using the State pools, as well as the type file to auto generate the domain file, you are only allowed to use elements provided in these two sources (state pools and type file), to write your Facts file. If you are not sure aboute which elements you configured, you can list them by clicking onto the 'State Pool Info' Button in the left corner of the Task Planner Plugin configuration window. 

**Example**

```
If Selecting /home/my_facts.pddl,
this file will be used as facts file during the Planning Process.
```

### Generate State Machine Into

The Plugin has two generation modi, one where a completely new State machine is created during a Task, and another where the Planning result is generated into an existing Hierarchy State.  
To select the mode, this field provides a radio button group. is 'independent State machine' was selected, the Plugin will create a new State machine, and if 'selected state' was choosen the Plugin will use an existing State to generate the planning result into.  
As the radio button name indicates, the plugin will try used the current selected State. Therefore it's mandatory, that exactly one state is selected, and that the selected State is a Hierarchy State.  
**Important**: To avoid accidents the plugin rejects non empty states. If you want to allow a particular state, to beused despite its not empty, you have to set the value of 'Allow_Override' in 'RAFCONTPP' in the Semantic Data Section of the State to 'True'. Then all child states of the Hierarchy State are automatically deleted before adding new ones.  
If 'selected State' is choose, the fields 'State machine name' and 'Save state machine in' are ignored.

**Example**

Value | Explanation
----|----
selected State | Generate into existing Hierarchy State.
independent State machine | Create new State machine.

### State Machine Name

The purpose of this text field is to enter the name, the resulting State machine will later have. If no name is provided, the name of the problem, defined in the facts file, is used instead. If 'selected State' in 'Generate State Machine Into' is choosen, this field is ignored.

**Example**

```
Example names are:
my_fancy_state_machine
planned_state_machine
task 42

```

### Save State Machine In

This field is a Directory chooser, and its purpose is, to set the path, where to save state machine, which is generated during the Task. If 'selected State' in 'Generate State Machine Into' is choosen, this field is ignored.  
**Important:** State machines with the same name, stored in the same path are overwritten.

**Example**

```
name: my_state_machine
Save State Machine In: /home/state_machines
With a configuration like this, the state machine my_state_machine will be stored in /home/state_machines
```

### Generated Files

During a Task some file are generated by the Plugin and the Planner. For example the Plugin will generate a Domain file, the Planner will (hopefully) generate a Plan, and maybe some other files. Usually they are not needed afterwards, but if you want to have a closer look, use them for debugging or need to keep them for logging purposes etc., you can tick the 'Save' Checkbox, and the files won't be deleted. If 'Save' is not ticked, all files are stored in the current working directory, and then deleted at the end of the Task.  

### Save Files In

## Runtime
### Runtime Data
### Include
## Buttons
### Generate State machine
### Cancel
### X In upper Right
### State Pool Info


