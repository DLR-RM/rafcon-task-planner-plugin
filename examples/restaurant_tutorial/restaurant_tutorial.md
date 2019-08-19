# 1. Restaurant Tutorial

This tutorial introduces the **basic functionality** of the Task Planner Plugin (Tpp), by leading through a little example.


- [1.1 Scenario description](#11-scenario-description)
- [1.2 Requirements / Pre-Setup](#12-requirements--pre-setup)
- [1.3 First Step](#13-first-step)
- [1.4 Preparing some States](#14-preparing-some-states)
  * [1.4.1 Eat](#141-eat)
  * [1.4.2 Cook](#142-cook)
  * [1.4.3 Give](#143-give)
  * [1.4.4 Move](#144-move)
  * [1.4.5 Order](#145-order)
- [1.5 Type Hierarchy](#15-type-hierarchy)
- [1.6 Facts Facts Facts](#16-facts-facts-facts)
- [1.7 Planning a State machine](#17-planning-a-state-machine)
- [1.8 Solution](#18-solution)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


## 1.1 Scenario description

Our scenario takes place in a little pizzeria. There is Bob, the chef baking pizza, the waiter James serving the guests,
and one guest alice.   

![Restaurant_example_overview](doc/restaurant_tutorial_overview.jpg "Scenario Overview")

Lets assume Alice is really hungry, and wants to eat some pizza. In this tutorial, we want to create a state machine, to feed Alice, e.g. a state machine, that models the process of Alice ordering a pizza in the restaurant, Bob cooking, and James serving it, so that Alice is able to eat it.
 
## 1.2 Requirements / Pre-Setup

To be able to run this tutorial, you need: 

 - **RAFCON**: `pip install --user rafcon` orː `git clone git@rmc-github.robotic.dlr.de:common/rafcon.git` (for information about how to install rafcon see: [install rafcon](https://github.com/DLR-RM/RAFCON))
 - **The Rafcon Task Planner Plugin**ː `git clone git@rmc-github.robotic.dlr.de:moro/rafcon_task_planner_plugin.git`
 - **The Fast Downward Planning System**ː `pip install --user downward-dlr --no-compile` (evt. you have to install the wheel package first)
 
## 1.3 First Step 
At first, the plugin has to be registered in RAFCON. To do so, `[RAFCON̙-TASK-PLANNER-PLUGIN̠-REPOSITORY̠-PATH]/source/rafcontpp` has to be added to the `RAFCON_PLUGIN_PATH` environmental variable. If you want to use the auto Layout Plugin, also add its path. (See [RAFCON Docs](https://rafcon.readthedocs.io/en/latest/plugins.html))  
 
`RAFCON_PLUGIN_PATH=$RAFCON_PLUGIN_PATH:[RAFCON̙-TASK-PLANNER-PLUGIN̠-REPOSITORY̠-PATH]/source/rafcontpp`


If you did this successfully, RAFCON should now have an extra menu button called "Plan Task", and a new tab called "PDDL Action" at the right, below the Semantic Data tab. The image shows RAFCON with the Task Planner Plugin loaded.

![Restaurant_example_rtpp_loaded](doc/Rafcon_rtpp_loaded.png "Plugin loaded")

## 1.4 Preparing some States
All in all five States are involved in the process. You have to create and store them in a directory of your choice, from now on referenced as [your_folder]. 

### 1.4.1 Eat
To give Alice the ability to eat her pizza, we have to create the state eat. To do soː

1. Create a new state machine in RAFCON

2. Change its State Name to "Eat" and its Type to "ExecutionState".

3. Add the two Input Ports "food" and "person", both of type str (all input ports have to be of type str).

4. Insert the following code into the source editorː  
      ```python
      import time   
      def execute(self, inputs, outputs, gvm):
          self.logger.info(inputs['person']+' is eating '+inputs['food'])
          time.sleep(0.5)
          return 0
    
      ```
5. Now we have to open the PDDL Action Tab below the Semantic Data Section. (NOT the PDDL Action dictionary in the Semantic Data Section)

6. If you want, you can write a description text into the **description field**, its for your documentation.

7. Every State has a corresponding **PDDL Action**, which represents its semantic meaning. Insert the following action into the PDDL section of the tab. 
    ```Pddl
    (:action eat
       :parameters (?person - Person ?food - Food)
       :precondition (and (has ?person ?food))
       :effect (and (not (has ?person ?food))
                    (be-full ?person)
               )
    )
    ```     
    The action is called eat, and has also two parametersː ?person of type Person and ?food of type Food  
    **IMPORTANT**ː the names of the variables ?person and ?food have to match with the names of the state's input ports person and food. But not every parameter of the action has to have an corresponding input port in the state and vice versa. 

8. In further states you can click auto complete, (bewareː auto complete is just a best effort approach, it works fine in most cases (I don't know a case yet, where it doesn't work), nevertheless you should check the results.) but in this first state I will explain all fields.

9. In the **Predicates section** you have to insert the predicates, which are used by the action. In this case, the predicates are "be-full" and "has". They are used for example as "(has ?person ?food)". The complete predicate (we have to insert the complete predicate into the field) would be "(has ?person - Person ?food - Food)".  
   In this section variable names don't matter, just the predicate name and its types. So (has ?person - Person ?food - Food) and (has ?dosent - Person ?matter - Food) are the same predicate, but (has ?person - Object ?food - Food) would be a different one (because it differs in the type "Object").  
   According to this, we have to insert the following into the Predicate Sectionː
    ```Pddl
    (be-full ?person - Person)
    (has ?person - Person ?food - Food) 
    ```   

10. In the **Types field**, we have to list all types which are needed by the action. In our case we can insert the types "Food" and "Person" as comma separated string.
    ```Pddl
    Food, Person
    ``` 
11. The **Requirements Section** is a bit complicated, and not all of them are decidable at this point in time. If you want to know, when to check which box, please follow the PDDL standard. In this case we have to check the "ːstrips" and the "ːtyping" box. "ːstrips" because we use conjunction and negative effects, and "ːtyping" because we use types e.g. Food and Person.
    ```
    check the fields: ːtyping and ːstrips
    ```
12. Save the State into [your_folder]. 

Once you are finish your state should look like in the figure below.

![restaurant_example_state_eat](doc/Rafcon_rtpp_eat.png "State Eat")


Repeat this process with the following statesː 

### 1.4.2 Cook

Giving Bob the ability to cook in the kitchenː 

**StateNameː** `Cook`  
**Typeː** `ExecutionState`  
**Input Portsː** `chef, food` (type str)  
**Source Editorː** 
```python
import time
def execute(self, inputs, outputs, gvm):
   self.logger.info(inputs['chef']+' is cooking '+inputs['food'])
   time.sleep(1.5)
   return 0
```
**Pddl actionː**
```Pddl
(:action cook
   :parameters (?chef - Chef ?food - Food)
   :precondition (wants ?chef ?food)
   :effect (and (has ?chef ?food)(not(wants ?chef ?food)))
)
```

**Other fieldsː** Hit Auto Complete

### 1.4.3 Give

give someone the ability to handover foodː  


**StateNameː** `Give`  
**Typeː** `ExecutionState`  
**Input Portsː** `interlocutor1, interlocutor2, food` (type str)  
**Source Editorː** 
```python
import time
def execute(self, inputs, outputs, gvm):
   self.logger.info(inputs['interlocutor1']+' gives '+inputs['food']+' to '+inputs['interlocutor2'])
   time.sleep(0.5)
   return 0
```
**Pddl actionː**
```Pddl
(:action give
   :parameters (?interlocutor1 ?interlocutor2 - Person ?food - Food ?loc - Location)
   :precondition (and (at ?loc ?interlocutor1)
                      (at ?loc ?interlocutor2)
                      (wants ?interlocutor2 ?food)
                      (has ?interlocutor1 ?food)
                 )
   :effect (and 
           (not (wants ?interlocutor2 ?food))
           (not (has ?interlocutor1 ?food))
           (has ?interlocutor2 ?food)
           )
)
```
**Other fieldsː** Hit Auto Complete

### 1.4.4 Move
Give the James the ability to move aroundː 

**StateNameː** `Move`  
**Typeː** `ExecutionState`  
**Input Portsː** `destination, name` (type str)  
**Source Editorː** 
```python
import time
def execute(self, inputs, outputs, gvm):
   self.logger.info(inputs['name']+' is moving to '+inputs['destination'])
   time.sleep(0.5)
   return 0

```

**Pddl actionː**
```Pddl
(:action got-to
   :parameters (?start ?destination - Location ?name - Waiter)
   :precondition (and (at ?start ?name))
   :effect (and (not (at ?start ?name))(at ?destination ?name))
)
```

**Other fieldsː** Hit Auto Complete

### 1.4.5 Order

Give Alice the ability to order her pizzaː 

**StateNameː** `Order`  
**Typeː** `ExecutionState`  
**Input Portsː** `food, interlocutor1, interlocutor2` (type str)  
**Source Editorː** 
```python
import time
def execute(self, inputs, outputs, gvm):
   self.logger.info(inputs['interlocutor2']+' is ordering '+inputs['food']+' from '+inputs['interlocutor1'])
   time.sleep(0.5)
   return 0
```

**Pddl actionː**
```Pddl
(:action order
   :parameters (?interlocutor1 ?interlocutor2 - Person ?food - Food ?loc - Location)
   :precondition (and (at ?loc ?interlocutor1)
                      (at ?loc ?interlocutor2)
                      (wants ?interlocutor2 ?food)
                 )
   :effect (and 
               (wants ?interlocutor1 ?food)
           )
)
```

**Other fieldsː** Hit Auto Complete


Once your are finish, you should have a folder, containing this five states. 

## 1.5 Type Hierarchy
 As you may have noticed, we are using types in the pddl actions. But you don't know the hierarchy of these types yet, and the plugin does that neither. Therefor we need a separate file, where this hierarchy is specified. Let's call it **rtpp-typehierarchy.json** it contains the followingː 
 
 ```json
{
"Person":"Object",
"Food":"Object",
"Location":"Object",
"Chef":"Person",
"Waiter":"Person",
"Guest":"Person"
}
```

The structure of the file is a dictionary with types as keys, and their parents as values. For example, the type "Waiter" is extending the type "Person", and "Person" is extending "Object", which is the root type and has no parent. 

## 1.6 Facts Facts Facts

As final step, we have to describe our problem, or better our goal, our initial state (init), and all "objects", which are present in our world. In this case the **objects** are James the Waiter, Bob the chef, Alice the Guest, a pizza, the kitchen, the table and the entrance of the restaurant. At the beginning of our situation (**init**) Bob is where chefs areː in the kitchen. James is standing at the entrance, waiting for new guests and Alice is sitting at a table. She is really hungry and wants a pizza. The **goal** is to bring Alice her pizza and make her feel full. Written in pddl, this could look like thisː 
```Pddl
(define (problem alice_is_hungry)(:domain restaurant)
(:objects 
   james - Waiter
   bob - Chef
   alice - Guest
   pizza - Food
   kitchen table entrance - Location
)
(:init
   (at entrance james)
   (at table alice)
   (at kitchen bob)
   (wants alice pizza)
)
(:goal (and
   (be-full alice)
   )
))
```

Copy it, and save it in a file, **facts.pddl** 

## 1.7 Planning a State machine

Now we are ready. We haveː 
1. A folder containing five prepared states
2. A rtpp-typehierarchy.json file, containing the type hierarchy
3. A facts.pddl file, containing our problem

Let's hit the "Plan Task" Button:

**State poolsː** This should contain the folder, where you saved the prepared states in.  
**Type fileː** Enter the location of your rtpp-typehierarchy.json file here  
**Plannerː** Choose the Fast Downward Planning System (you installed it at the beginning of this tutorial)  
**Planner script locationː** Leave it at it is (its not considered as long as you don't choose "Other..." in the Planner field.). If you want to use your own planner with the Plugin, you can enter the location of your integration script here.  
**planner argvː** Leave it empty. If you want to pass arguments directly to the planner, you can enter them here.  
**Facts fileː** Enter the location of your facts.pddl file.  
**Generate State machine Into:** Select independent State machine
**State machine nameː** Let's call it restaurant_state_machine  
**Save state machine inː** Enter the location, where the plugin should save the state machine it will generate.  
**Generated filesː** The Plugin will create additional files, like a domain, or the plan to solve the problem. if you want to keep them, check this button.  
**Save files inː** Specifies the location, where to save generated files.  
**Runtime Dataː** Leave it empty.  
**Includeː** As long as Runtime Data is empty, this buttons are not considered, so we don't need to change something here.   

![Restaurant_example_plugin_conf](doc/Rafcon_rtpp_configuration.png "Plugin Configuration")

If we hit the "Generate State machine" button now, the planner will try to solve the scenario. Afterwards the found plan will be used to create a state machine.
![Restaurant_example_state_machine](doc/Rtpp_restaurant_statemachine.png "Restaurant State machine")

If we execute the state machine now, the console output should look like below, as you can see Alice is getting her pizzaǃ 
```consoleOutput
INFO - Go to: james is moving to table
INFO - Order: alice is ordering pizza from james
INFO - Go to: james is moving to kitchen
INFO - Order: james is ordering pizza from bob
INFO - Cook: bob is cooking pizza
INFO - Give: bob gives pizza to james
INFO - Go to: james is moving to table
INFO - Give: james gives pizza to alice
INFO - Eat: alice is eating pizza 
```


You can't just plan this one state machine, but arbitrary manyǃ For example, let's assume Alice brought her friend Eve, who wants a soup, and Bob who wants to eat a sandwich at work. The facts file would look like below, but feel free to play around, test other scenarios, and try to plan them. 
```Pddl
(define (problem serve_guest) (:domain restaurant)
(:objects 
   james - Waiter
   bob - Chef
   alice eve - Guest
   pizza soup sandwich - Food
   kitchen table entrance - Location
)
(:init
   (at entrance james)
   (at table alice)
   (at table eve)
   (at kitchen bob)
   (wants alice pizza)
   (wants eve soup)
   (wants bob sandwich)
)
(:goal (and
   (be-full alice)
   (be-full eve)
   (be-full bob)
   )))
```
## 1.8 Solution

A solution for this tutorial is provided in `[repository_path]/examples/restaurant_tutorial/src`.
There you find all described files, and states as well as an executable restaurant_state_machine and all related files, generated during the planning process.
