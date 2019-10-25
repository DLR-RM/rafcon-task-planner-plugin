---
layout: default
title: Getting Started
---

# Getting Started

## Installation
The plugin is currently available on two platforms. To install it from one of these, two steps are required:<br>

### **Intall from GitHub**
1. **Clone the git repository** <br>
At first the plugin hast to be cloned from the [Git Repository](https://github.com/DLR-RM/rafcon-task-planner-plugin) into a directory of your choice:<br>
`git clone https://github.com/DLR-RM/rafcon-task-planner-plugin.git`<br>

1. **Register the Plugin in RAFCON**<br>
To register the plugin in RAFCON, it's plugin path has to be added to RAFCON's plugin path variable. The plugin path is `[repository_path]/source/rafcontpp`, so adding it would look like:<br>
`RAFCON_PLUGIN_PATH=$RAFCON_PLUGIN_PATH:[repository_path]/source/rafcontpp`<br>
More information about, how to add plugins to RAFCON can be found in the [RAFCON Documentation](https://rafcon.readthedocs.io/en/latest/plugins.html).<br>

### **Install form PyPI**
**Important:** The PyPI version doesn’t contain any examples!<br>
1. **Obtain the Plugin**<br>
The Plugin can be downloaded using pip:<br>
`pip install rafcon-task-planner-plugin`<br>

1. **Register the Plugin in RAFCON**<br>
To register the plugin in RAFCON, it's plugin path has to be added to RAFCON's plugin path variable. The plugin path is `[site-packages_path]/rafcontpp`, so adding it would look like:<br>
`RAFCON_PLUGIN_PATH=$RAFCON_PLUGIN_PATH:[site-packages_path]/rafcontpp`<br>
More information about, how to add plugins to RAFCON can be found in the [RAFCON Documentation](https://rafcon.readthedocs.io/en/latest/plugins.html).<br>


If the steps were executed correctly, RAFCON should load the plugin on it's next start. The best indicator that everything worked well is, that the GUI should have a new Button ["Plan Task"](PlanTaskButton.md) in the Menu Bar.<br>
After downloading a planner, next steps could be to play around with the plugin, read more [documentation](../documentation.md), or start with the [tutorials](../tutorials.md). 

## Dowloading a Planner
The plugin dosen't work without a semantic planner. How to download one, and prepare it for RTPP is described in the [Built-in Planner Section](Planner.md). It's recomended, to start with The Fast Downward Planning System.

