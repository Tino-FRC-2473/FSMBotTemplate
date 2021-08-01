# Lesson 1: Introduction to WPILib

This lesson will go over the basics of using WPILib in VSCode.

To access this lesson, clone the project using git:

`git clone https://github.com/Tino-FRC-2473/FSMBotTemplate.git`

Then checkout the branch for the lesson

`git checkout training/lesson1`

## Navigating projects in VSCode
Open VSCode. Then from `File->Open Folder`, select the top level folder containing the training project. This should load up the project in VSCode.

You can open files in the project using `Ctrl+P` shortcut. The shortcut will open a search box which you can use to quickly find and open files.

Let's try this out. Search for this markdown file, `Lesson1.md` and open it.

VSCode includes a bunch of useful plugins for performing common tasks. These can be accessed from the "Command Palette" by pressing `Ctrl+Shift+P` shortcut. For instance, with a markdown file like this one open, open the command palette and search for `Markdown: Open preview`, and select that option. This should open a nicely formatted previw of this mardown document.

## Using WPILib in VSCode
WPILib has automated the build and deployment process as plugins in VSCode. Let's take a look at some common tasks now. For more details, see [WPILib documentation]()

### Building code
After you have written some code, you'll have to compile it before it can be run. If you open the command palette, you'll find this step as `WPILib: Build Robot Code`. This will trigger the Java compilation process and present the results in the output tabs in the bottom section of VSCode.

In the `teleopPeriodic()` method of Robot.java, there's a "Hello World!" print. Change the message to something different and try building the project.

### Deploying code
Now that the code has been compiled, we need to get it onto the robot. From the command palette, this can be done by running the "WPILib: Deploy Robot Code" command while connected to the robot's Wi-Fi network. This will copy the compiled code over to the robot and start the new program. By default this will also open up the Riolog viewer showing logs from the robot.

### Simulating code
What if we want to test robot code but the hardware team is busy working on a mechanism? WPILib recently added an option to run the robot code right on your computer in a simulation environment.

From the command palette, select the `WPILib: Simulate Robot Code on Desktop` command. Then, make sure the `halsim_gui` option is checked to launch the simulation Glass UI, and press "OK".

Read through the [WPILib docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html) and make sure you are familiar with all the parts of the simulation UI.

Try switching the simulation into Teleop mode. Can you see the print you added in the console output?

### Debugging code

Modern IDE's like VSCode come with powerful debugging capabilities that can help track down issues much faster than sprinkling `println` calls everywhere.

One of the key debugging features is the ability to set breakpoints. Breakpoints let you mark specific lines of code directly in the editor. When the robot reaches that line of code, it will pause execution and hand over control to VSCode. This lets you inspect variables and even run code line by line to see if your code is doing what you expect it to do.

While running the example project earlier, you might have noticed a "Hello!" print occasionally popping up. Let's take a look at how this is happening. Go to FSMSystem.java, and find the `counterPrint()` method on line 86. If you hover on the margin to the left of the line number, a red dot will appear. Click on that red dot to set a breakpoint at that line of code (see screenshot [here](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/debugging-robot-program.html#setting-a-breakpoint)). Make sure your breakpoint is on the line of code inside the function, not the function declaration.

Now that the breakpoint is set, let's start the debugging session. If you are connected to a real roboRio, select `WPILib: Debug Robot Code` option from the command palette. If you are in a simulation, debug mode is opened automatically.

When running in debug mode, the Debug panel will open on the left sidebar of VSCode. Now switch into teleop mode, and run until you hit the breakpoint. VSCode will highlight the line the robot is on in the editor. In the debug sidebar, you can see the current values of all objects and their variables. At the bottom, you can see the call stack, showing all the functions called to get to this point.

At the middle top, there should be a bar with the basic [Debug Actions](https://code.visualstudio.com/docs/editor/debugging#_debug-actions) for controlling the program. From left to right, these are:
* Continue / Pause (F5): Pause or unpause code. In particular, if you are in a breakpoint, Continue tells the code to keep running until it hits the next breakpoint.
* Step Over (F10): Run the next line of code
* Step Into (F11): If there is a function, go into the implementation of that function.
* Step Out (Shift+F11): Run until the current function returns.
* Restart (Ctrl+Shift+F5): Stop and rerun code. This doesn't really work with WPILib.
* Stop (Shift+F5): Stop the code. This only really works with simulation.

Try stepping through this program to get a feel for debugging with VSCode. In particular, if you "Continue" multiple times, what pattern do you see in the value for counter?

Debugging is very powerful for tracking down bugs quickly and efficiently. VSCode has a whole bunch of other features like the debug console and watchpoints that can do even more.

One last important point is to be very careful when setting breakpoints on the real robot. In particular, if some motor is moving when a breakpoint is hit, it may keep moving while you are debugging, since robot code is paused! Remember that the driver station is always able to disable the robot, even if it is being debugged.
