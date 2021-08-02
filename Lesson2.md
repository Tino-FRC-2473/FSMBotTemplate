# Lesson 2: Working with Finite State Machines

This lesson will go cover the basics of using FSMSystems.

To access this lesson, clone the project using git:

`git clone https://github.com/Tino-FRC-2473/FSMBotTemplate.git`

Then checkout the branch for the lesson

`git checkout training/lesson2`

## Introduction
For this lesson, we'll be creating a simple light switch using the FSMSystem. In particular, we'll make this as simple as possible by having one button on the controller turn the light on and another button on the controller turn the light off.

## Hardware Definition
The first step is to define and intialize the hardware that our light switch system will be controlling. For this example, the light will be turned on an off by a relay controlled by a [DigitalOutput](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/DigitalOutput.html) channel on the roboRio.

A DigitalOutput called `lightRelay` has already been defined for you in `LightSystem`. However, it still needs to be instantiated. Assign a channel number for this digital output in `HardwareMap` and add the instantiation of `lightRelay` in LightSystem's constructor. Remember that the channel number is a constant, so mark it `static final`.

# Input Definition
Next, let's choose the inputs we want to use. We'll be using the buttons on a [Joystick](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Joystick.html) to turn our light on and off. In TeleopInput.java, a Joystick has already been created and instantiated for you. You just need to fill out the getter methods `isOnButtonPressed()` and `isOffButtonPressed()`. Remember which button IDs you used for on and off, we'll need this for testing.

## State machine definition
Finally, let's create a state machine for this simple system. There's obviously two possible output states here, either the light is on, or the light is off. Come up with names for these two states, and add them in the FSMState enum in LightSystem.java. Also create the state handler functions that define what the output should be for each state, and make sure they are called in `update()`. Hint: [DigitalOutput.set()](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/DigitalOutput.html#set(boolean)) might be useful.

Next, we have to choose the start state for this system. Think about what would make sense as the starting state, and make sure `reset()` sets the system to the correct starting state.

Finally, define the state transitions in `nextState()`. You can access button values as needed via `input`. Remember to handle all cases! Hint: if no buttons are pressed, does the state change?

## Scheduling
Now that our system is defined, let's add it to the scheduling methods in Robot.java. First, make sure we instantiate our LightSystem when the robot boots in `robotInit`. Then update the teleop scheduling functions `teleopInit()` and `teleopPeriodic()` to reset and update our system as expected.

## Testing
When you think you have everything sorted out and your code builds succesfully, give it a run in the simulator. If you don't have a real Joystick connected to your computer, the Simulator lets you use keyboard keys as a substitute. See the [WPILib docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html#adding-a-system-joystick-to-joysticks) for a guide on how to do this. Select Hardware->DIO to show the simulated digital output values.

Switch into Teleop mode, and press the buttons you assigned to on and off. Does the light react as you expect?

Bonus: What happens if you press both buttons at the same time? And why?