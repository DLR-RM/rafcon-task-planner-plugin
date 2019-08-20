---
layout: default
title: PDDL Action Tab
---
# Pddl Action Tab

The Plugin introduces a new Tab in the State Editor, it's called the Pddl Action Tab, and its Symbol is a little Calculator.  
The Tabs gives the possibility to annotate a state with a [PDDL](https://en.wikipedia.org/wiki/Planning_Domain_Definition_Language) Action, and so to make it usable for the plugin and so for planning.

![Rafcon With the PDDL Action Tab](media/img/PDDLActionTab.png "A open RAFCON window with the PDDL Action Tab in the right.")


- [Tab Fields](#tab-fields)
  * [Description](#description)
  * [PDDL](#pddl)
  * [Predicates](#predicates)
  * [Types](#types)
  * [Requirements](#requirements)
- [Buttons](#buttons)
  * [Auto Complete](#auto-complete)
  * [Apply](#apply)
  * [Enable Auto Apply](#enable-auto-apply)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


## Tab Fields

The Tab consists of several fields, describing the States semantic in PDDL. This section explains the fields in Detail. 

### Description

The Description field is a text field. It has Documentation purposes, and should contain a description and hints of the PDDL Action. So it is not used by the Plugin.

### PDDL

The Action in PDDL Syntax should be pasted or written (not recomended) into this code editor field. Information about what aspects of PDDL are supported by the Plugin can be found in the Section [PDDL Support](HomePage.md#pddl-support). When generating a Domain during a Task, this field is copy pasted one by one. It's also parsed during the "Auto Complete" process. so only valid PDDL Code, as well as valid PDDl Comments are allowed here.

**Example**

```PDDL
;That is my move action
(:action move
:parameters (?form ?to - Location ?subject - Vehicle)
:precondition (at ?from ?subject)
:effect (and (not (at ?from ?subject)) (at ?to ?subject))
)
```
### Predicates

The Predicates text field should be filled with all different Predicates, the Action in the 'PDDL' field uses. Due to the fact, that all predicates in the Action are applied, the parameter types have to be added. This field is the foundation of predicates section in the auto generated domain file. For the Plugins point of view, no type hierarchy is defined  at this point. So the list should not only contain all Predicates with different names, but also with the same name, and different types. (Btw. it's also not a good idea to merge predicates by hand, because the type hierarchy may change in the future.)

**Example**

```
According to the example above (the PDDL Action) the Predicates field would 
only contain one predicate: (at ?from - Location ?subject - Vehicle)


Another, imaginary Example would be: 

(at ?a - Location ?b - Vehicle)
(at ?a - Location ?c - Cargo)
(in ?c - Cargo ?b - Vehicle)
```


### Types

The types text field should contain all types, which are used in the Action in the 'PDDL' field. The types must be separated by spaces or commas. This field is used to decide which types the types section in the auto generated domain file will contain. 

**Example**
```
According to the example above (the PDDL Action) the Types field would look like this:

Location, Vehicle
```

### Requirements

This checkboxes represent requirements on PDDL Planners. For example, if an action uses Types (all actions in the Plugin do that), the Planner requirement would be ':typing'. All requirements the Action in the 'PDDL' field has should be ticked here.

## Buttons

The PDDL Action tab comes with a few Buttons. These are described in this Section.

### Auto Complete

The Auto complete Button is located between the 'PDDL' and the 'Predicates' field. As its title indicates, it tries to auto complete the Predicates, Types and Requirements fields, and applies all changes. First to mention: It tries. Currently no bugs are known, and usually it finds all Predicates, types and requirements needed, but there is NO guarantee. Second to mention: It's Auto Complete. Therefore it will just add, input to the fields, but not override them.  

### Apply

The Apply Button is located in the footer of the Action Tab. The Plugin stores all information of the PDDL Action in the "Semantic Data" Tab of vanilla Rafcon. so the Apply Buttons purpose is, transmit the entered data from the PDDL Action Tab into the Semantic Data Tab.    
**Important:** Hitting the Apply Button will not store the PDDL Data persistent. It is just persistent after saving the State. If the state was saved but the Apply button wasn't clicked, the PDDL Action Tab is also not saved.

### Enable Auto Apply

If Auto Apply is enabled, all changes are automatically applied. usually enabeling is recomended, but due to some performance issues, it should be disabled when writing a (long) description or coding in inside of the PDDL field.  
