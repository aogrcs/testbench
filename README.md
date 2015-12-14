# testbench_vhvc_indigo
CDV simulator-based testbench with test templates, v.hvc, with support for ROS Indigo

----------------CONTENTS------------------------------------------------

bert2_moveit ROS package --> path planning
bert2_simulator ROS package --> simulator and testbench
simulator_ *.sh --> bash scripts for the testing in batch mode.
Inside the bert2_simulator package --> BERT2 robot model, human model and object model, for Gazebo.
Inside bert2_simulator/scripts --> simulator nodes, assertion monitor nodes, coverage collector module, "stimulus" (test templates with high-level human actions) for pseudorandom, constrained and model-based test generation in requirements and cross-product coverage.
Inside bert2_simulator/scripts/testgens --> test generator modules, UPPAAL PTA model (6 PTA automata), CTL properties for model checking and model-based test generation.


----------------BEFORE USE---------------------------------------------- 
Assumptions:
- Full installation of ROS, Gazebo and MoveIt!. 
- Full installation of Python Coverage modules.
- Compilation of the packages. 
INSTRUCTIONS ARE PROVIDED IN INSTALL_INSTRUCTIONS.txt

This testbench version works on ROS Hydro and Gazebo 1.9, running in Ubuntu Precise, Quantal or Raring; and ROS Indigo and Gazebo 2.2, running in Ubuntu 14.04 (Trusty). 

For the model-based test generation, UPPAAL 4.0.14 (recommended), CoVer 1.4 http://www.hessel.nu/CoVer/. For an UPPAAL (TA or PTA) model (model.xml) and CTL properties saved in model.q, to generate traces (test templates in model-based test generation) from command line, do: 
./cover -t 0 -G -f output_file_name model.xml model.q 
This will generate a number of traces (one for each property in the model.q file) with the file name output_file_name-#.xtr. Then, use the the provided Python scripts (in /scripts/testgens) to translate into suitable stimulus files. 

----------------RUNNING THE TESTBENCH-----------------------------------
For a single experimental run, follow SINGLE_RUN_INSTRUCTIONS.txt

For bash mode, run the bash scripts from the directory ~/ (your home). Each script runs tests with stimulus from model-based, random, constrained test generation. Reports are generated in the /tmp/ folder. Reports on assertion coverage and code coverage are produced automatically in ~/catkin_ws. 

Questions, bugs, comments: dejanira.araizaillan@bristol.ac.uk, david.western@bristol.ac.uk
