# DesignProject03
Design Project #3 for AE 353: Aerospace Controls

# Goal: 
Design a controller that will control the pitch angular (phi_dot) of a glider that has no thrusters. I want the glider to glide over an average of 20 ft (will be tested over 1,000 trials). The test is how well my glider can adjust to random initial conditions. 

*Since I do not have sensors measuring all parts of my state, I must use state-estimation methods to readjust my actuator*

# To run:
Open Files in MATLAB

In the command window run: DesignProblem02('Controller','diagnostics',true)

# File Descriptions:
------------------
-Main.pdf: PDF of submitted report.

-DesignProblem03: Provided simulation that creates a glider with random initial flight conditions.

-Controller: Controller that is called by DesignProblem03 to control glider

-workspace: Workbench
*Also includes sections that plot behavior of all 1,000 flight simulations that are shown in the report.*

-data.mat Data generated from Designproblem03
