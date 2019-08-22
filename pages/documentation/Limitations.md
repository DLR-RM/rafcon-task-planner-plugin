---
layout: default
title: Limitations
---

# Limitations

This section is about the RTPP's limitations. Since the plugin is still in development, they may change over time. 

## Scenarios

- **Linear Scenarios only**<br>
Currently the plugin can only generate linear state machines e.g. tubes. Therefore it can't handle scenarios where concurrency is needed. So if tasks with more than one actor are planned (as it is the case in this [tutorial](../tutorials/turtle_sim_example.md)), only one actor will be active at a time. 
<br><br>
- **No consideration of costs during Planning**<br>
Real world robots have limited ressources like fule or battery power. According to the plugin's current [PDDL support](#pddl-support), these costs can't be modeled, and therefore not considered during the state machine generation process. So it should be ensured, that executing the generated state machine is feasible with the ressources available. 
<br><br>
- **Number of Objects and Skills**<br>
Domain independent planning is still costly in terms of time and computational power. These costs depend on the number of objects present, and number of skills executable in a specific scenario. Reliant on the planner, this dependency is exponential, so that adding one object could in some cases change the planning duration from minutes to hours. 

## PDDL Support

Since PDDL is extensive, we decided to provide a minimal PDDL set in the beginning, and extend our support on demand. So please feel free to contact us in order to increase our Pddl support. Currently RTPP supports: 

- Only actions in "classical" :parameter - [:precondition] - :effect pattern, without functions / fluents, or cost consideration.
- Due to the fact that the resulting state machines are executed in the real world, untyped variables are not supported.
