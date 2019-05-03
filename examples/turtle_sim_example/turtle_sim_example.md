# Turtle Sim Example

This example should demonstrate how the **data flow** can work in a planned state machine and how **flexible** they are even when using them with **Ros**.
The example is base on rafcons [turtle demo](https://rafcon.readthedocs.io/en/latest/tutorials.html#starting-the-basic-turtle-demo-state-machine-using-ros), but the states where modified to use the plugins data flow style, and enriched with PDDL actions.

## Scenario description
The scenario consists of:
- a map with five points (down_left, down_right, top_left, top_right and middle) 
- and three turtles (alice, bob and eve).



With the following rules:
- turtles can only move between connected points
- if a turtle moves it gets hungry
- if a turtles are hungry they can eat eachother








