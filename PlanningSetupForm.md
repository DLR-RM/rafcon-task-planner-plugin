
# The Planning Setup Form

INTERT PICTURE

The Planning setup form can be opened by clicking the "Plan Task" Button in the Menu bar of [RAFCON](https://dlr-rm.github.io/RAFCON/).  
It's the main Window of the Plugin, and the part where to configure a new Task, or get information about the current configuration.

## Planning Section
The Planning section contains all fields relevant for Planning and the State machine generation process. This section explains the fields in Detail.

### State pools
INSERT PICTURE
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
Important: All your types used in the annotated States, or in your facts file have to be part of this hierarchy.

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
This 
### Planner Script Location
### Planner Argv
### Facts File
### Generate State Machine Into
### State Machine Name
### Save State Machine In
### Generated Files
### Save Files In
## Runtime
### Runtime Data
### Include

