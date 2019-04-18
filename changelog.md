### Version 1.3
- added tooltips for a better understanding
- added a state machine name into the setup form, so that the state machine that will be generated can now be named freely
- added a possibility to add runtime data into the state machine, which will be stored in a global dictionary in rafcon, when the   state machine is executed. Therefore the Data can be included directly into the state machine, or as reference (asfile path), and be read during runtime. (the data has to be provided as (nested) json dictionary)
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
