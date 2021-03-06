### Version 1.5
- Plan Into state: Now its possible to plan a state machine into a selected hierarchy state of an existing one!
- Layouter: added a layouter, which formats the state machine after its generation!
- Restructured the semantic data section (the tab is saved in)
- Added a "allow_override" permission flag for the plan into state feature to avoid accidents.
- The runtime data of a state machine is now not set, but added to the global dictionary, this allows multiple planned state machines with runtime data to be executed at the same time, or use data from previous ones.
- Ported the restaurant tutorial to git.
- Added a Turtle Sim Example, to explain how the runtime data feature works.
- updated documentation.
- Fixed major and minor bugs
- Added a state pool info view, which allows to show a summary of all configured PDDL data e.g. which pddl actions are inside of the cofigured state pool, which types are configured, and which predicates are available to write a facts file. 
### Version 1.4
- added a chooser entry for the runtime data
- added a state pool(s) info window
- updated documentation
- adjusted examples
- fixed some bugs
### Version 1.3
- added tooltips for a better understanding
- added a state machine name into the setup form, so that the state machine that will be generated can now be named freely
- added a possibility to add runtime data into the state machine, which will be stored in a global dictionary in rafcon, when the   state machine is executed. Therefore the Data can be included directly into the state machine, or as reference (asfile path), and be read during runtime. (the data has to be provided as (nested) json dictionary)
- started to parse the facts file, in order to get consistent object names. Some planner change 'pizza' indo 'PIZZA' because pddl is case insensitive. To prevent that, variable names in the plan are later exchanged with the original definition in the facts file.
- removed some minor and major bugs e.g. a gui freeze bug
- wrote documentation
- wrote unittests
- refactored some modules
- tried to do some code beautification
### Version 1.2
- added the possibility to interrupt a running planning task
- fixed some bugs
### Version 1.1
- added changelog
- integrated Fast-Forward Planning System
- added possibility to change between auto apply and manual apply in action tab
- added availability check for built in planners (planners get a hint in configuration list, if not available in the system)
- changed button name: "Task Planner Plugin" --> "Plan Task"
- made the planning process async
- added an indicator to Plan Task button, to show how many planning Tasks are in process
- excluded pddl comments from beeing parsed as valid pddl by the plugin
- improved the auto complete feature in pddl action tab
- fixed the predicate merge bug
- fixed some bugs

### Version 1.0
- implemented base features
- integrated Fast Downward Planning System
- added Pddl Action Tab
- Added "Task Planner Plugin" menu button
