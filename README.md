# UAM
Urban Air Mobility Individual Research Project Thesis. MSc in Aerospace Dynamics at Cranfield University. Code is based on the thesis, provided as documentation. Written using MATLABv2019b, Simulink and Simscape. 

**AVL** folder includes the documentation needed to run AVL alone. Can be used for any aircraft aerodynamic evaluation within the limits of vortex lattice method. Script_AVL.m is the main script, everything can be run from there.

**Optimization** folder includes the assembly of the model for optimization, including propellers and different corrections. Main_Script.m to run optimization. Once aircraft is configured, Trimming3D.m obtain the trimming conditions. Additional trimming and performance analysis scripts can be provided by request.

**Model** of the full aircraft does not include battery as it was not necessary for implementation in the flight simulator. Powerplant with battery model can be found in the battery folder inside Model. Run Initialization_UAM_v2 to obtain all the necessary data. Model defined in Urban_Air_Mobility_v3.slx. For additional scripts for simulation (e.g. flight condition definition) please contact Gonzalo.

**Response surface** includes the scripts necessary to create the metamodel/response surface. Use of regression and neural networks. Response_surface_creation.m is the main script, an example batch of data is provided to run some tests. The optimization domain is quite narrowed in this batch, therefore regression would provide reasonable accuracy without the need of neural networks.

Additional scripts can be provided by the author. In case of any doubt or further enquiries, please contact at gonzalovaldescernuda@gmail.com 
