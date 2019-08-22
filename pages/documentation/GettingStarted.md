---
layout: default
title: Getting Started
---

# Getting Started

## Installation
To install the RTPP, currently two steps are required:<br>
1. **Clone the git repository** <br>
At first the Plugin hast to be cloned from the [Git Repository](https://github.com/DLR-RM/rafcon-task-planner-plugin) into a directory of your choice:<br>
`git clone https://github.com/DLR-RM/rafcon-task-planner-plugin.git`<br>

1. **Register the Plugin in RAFCON**<br>
To register the plugin in RAFCON, it's plugin path has to be added to RAFCON's plugin path variable. The plugin's path is `[repository_path]/source/rafcontpp`, so adding it would look like:<br>
`RAFCON_PLUGIN_PATH=$RAFCON_PLUGIN_PATH:[repository_path]/source/rafcontpp`<br>
More information about, how to add plugins to RAFCON can be found in the [RAFCON Documentation](https://rafcon.readthedocs.io/en/latest/plugins.html).<br>

If all steps are executed correctly, RAFCON should load the plugin on it's netxt start. The best indicator that everything worked well is, that the GUI should have a new Button "Plan Task" in the Menu Bar.<br>
Next steps could be to play around with the plugin, read more [documentation](../documentation.md), or start with the [tutorials](../tutorials.md). 
