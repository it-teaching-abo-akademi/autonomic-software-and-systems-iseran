# Autonomic-Software and Systems - iseran
autonomic-software-and-systems-iseran created by GitHub Classroom

0. Make sure that you have python and pip installed on your PC

1. Download precompiled version of carla (0.9.4) and follow the instructions at: https://carla.readthedocs.io/en/latest/getting_started/

2. Check that it works by running CarlaUE4, which should open the simulation window

2.1 If at this point it complains that you do not have opengl drivers, you should check that you have newest graphics drivers installed

3. After the simultor is running, use the command: python spawn_npc.py -n 40

3.1 If you are on the windows, you should install anaconda, open anaconda command prompt and navigate into folder where you unpacked carla

At this point you should have a simulation window with 4 cars in it that drive on autopilot. If that is the case, it means you succesfully launched carla and all libraries are working. Running manual_control.py will create a car that you can control inside the simulation enviroment. You can see the usage of carla API inside tutorial.py and manual_control.py (their documentation is not yet done properly, but code is well commented).

4. Download the Excercise.zip and unpack all of it inside the same folder that other carla scripts are.

The usage of spawn_custom_npc is same as spawn_npc. The only difference is that instead of using hardcoded ai from carla, it uses the custom classes that we created for this excercise. spawn_custom_npc script uses custom_ai.py as controller and driver for the cars. Your task will be to implement missing functionality from subclasses contained in custom_ai.py, ai_parser.py and ai_control.py. The required functionality is marked by 'TODO' comments in the code that explain the missing functionality.
You can (and should as that will speed up testing time) use spawn_custom_npc and manual_control as basis to write your own script for testing the implemented functionality.

5. First familiarize yourself with the architecture and things that need implementing. Afterwards start filling the ai_control.py by first implementing movement for Executor class.

If you need to add or change parts of the program you are free to do so, but you should try to stay inside the confines of the selected model (ie. the modules are clearly divided into knowledge, monitor, analyser, planner and executor with autopilot class managing them). As 'Knowledge' is main interaction point between classes, it is expected that most classes use it, so you need to make usre it's architecture stays as robust as possible (which is not always possible) to accomodate new additions without getting too clogged.

6. Afterwards you should implement simple planner, so that you can define a route for the car (leave crash handling for later). If planner requires data from analyser, then implement mock functions with empty output.

7. Implement the Analyser and Monitor classes (the barebones version is inside ai_parser.py) so that you get the data from Carla and pasrse it, so that planner can do educated planning.

8. Finally go back to planner and implement crash handling.
