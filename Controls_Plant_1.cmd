!
!-------------------------- Default Units for Model ---------------------------!
!
!
defaults units  &
   length = meter  &
   angle = rad  &
   force = newton  &
   mass = kg  &
   time = sec
!
defaults units  &
   coordinate_system_type = cartesian  &
   orientation_type = body313
!
!--------------------------- Model Specific Colors ----------------------------!
!
!
if condition = (! db_exists(".colors.COLOR_R165G158B149"))
!
color create  &
   color_name = .colors.COLOR_R165G158B149  &
   red_component = 0.6470588235  &
   blue_component = 0.5882352941  &
   green_component = 0.6196078431
!
else 
!
color modify  &
   color_name = .colors.COLOR_R165G158B149  &
   red_component = 0.6470588235  &
   blue_component = 0.5882352941  &
   green_component = 0.6196078431
!
end 
!
if condition = (! db_exists(".colors.COLOR_R212G171B032"))
!
color create  &
   color_name = .colors.COLOR_R212G171B032  &
   red_component = 0.831372549  &
   blue_component = 0.1294117647  &
   green_component = 0.6705882353
!
else 
!
color modify  &
   color_name = .colors.COLOR_R212G171B032  &
   red_component = 0.831372549  &
   blue_component = 0.1294117647  &
   green_component = 0.6705882353
!
end 
!
if condition = (! db_exists(".colors.COLOR_R136G089B086"))
!
color create  &
   color_name = .colors.COLOR_R136G089B086  &
   red_component = 0.537254902  &
   blue_component = 0.337254902  &
   green_component = 0.3490196078
!
else 
!
color modify  &
   color_name = .colors.COLOR_R136G089B086  &
   red_component = 0.537254902  &
   blue_component = 0.337254902  &
   green_component = 0.3490196078
!
end 
!
!------------------------ Default Attributes for Model ------------------------!
!
!
defaults attributes  &
   inheritance = bottom_up  &
   icon_visibility = on  &
   grid_visibility = off  &
   size_of_icons = 5.0E-02  &
   spacing_for_grid = 1.0
!
!--------------------------- Plugins used by Model ----------------------------!
!
!
plugin load  &
   plugin_name = .MDI.plugins.controls
!
!------------------------------ Adams View Model ------------------------------!
!
!
model create  &
   model_name = SCARA
!
view erase
!
!-------------------------------- Data storage --------------------------------!
!
!
data_element create variable  &
   variable_name = .SCARA.Q1  &
   adams_id = 1  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .SCARA.Q2  &
   adams_id = 2  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .SCARA.GAMMA1  &
   adams_id = 3  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .SCARA.GAMMA2  &
   adams_id = 4  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .SCARA.Px  &
   adams_id = 5  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .SCARA.Py  &
   adams_id = 6  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .SCARA.QP1  &
   adams_id = 7  &
   initial_condition = 0.0  &
   function = ""
!
data_element create variable  &
   variable_name = .SCARA.QP2  &
   adams_id = 8  &
   initial_condition = 0.0  &
   function = ""
!
!--------------------------------- Materials ----------------------------------!
!
!
material create  &
   material_name = .SCARA.steel  &
   adams_id = 1  &
   density = 7801.0  &
   youngs_modulus = 2.07E+11  &
   poissons_ratio = 0.29
!
!-------------------------------- Rigid Parts ---------------------------------!
!
! Create parts and their dependent markers and graphics
!
!----------------------------------- ground -----------------------------------!
!
!
! ****** Ground Part ******
!
defaults model  &
   part_name = ground
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.ground
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .SCARA.ground.MARKER_1  &
   adams_id = 1  &
   location = 0.0, 0.0, -0.1298991354  &
   orientation = 0.0, 0.0, 0.0
!
part create rigid_body mass_properties  &
   part_name = .SCARA.ground  &
   material_type = .SCARA.steel
!
part attributes  &
   part_name = .SCARA.ground  &
   name_visibility = off
!
!------------------------------------ BASE ------------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.ground
!
part create rigid_body name_and_position  &
   part_name = .SCARA.BASE  &
   adams_id = 2  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.BASE
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .SCARA.BASE.PSMAR  &
   adams_id = 11  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
marker attributes  &
   marker_name = .SCARA.BASE.PSMAR  &
   visibility = off
!
marker create  &
   marker_name = .SCARA.BASE.cm  &
   adams_id = 12  &
   location = 0.0, 0.0, -0.1298991354  &
   orientation = 4.7123889804, 1.5707963268, 1.5707963268
!
marker create  &
   marker_name = .SCARA.BASE.MARKER_2  &
   adams_id = 2  &
   location = 0.0, 0.0, -0.1298991354  &
   orientation = 0.0, 0.0, 0.0
!
marker create  &
   marker_name = .SCARA.BASE.O1  &
   adams_id = 3  &
   location = 0.0, 0.0, -2.5E-02  &
   orientation = 0.0, 0.0, 0.0
!
marker create  &
   marker_name = .SCARA.BASE.MARKER_18  &
   adams_id = 18  &
   location = 0.0, 0.0, -2.5E-02  &
   orientation = 0.0, 0.0, 0.0
!
part create rigid_body mass_properties  &
   part_name = .SCARA.BASE  &
   density = 2700.0
!
! ****** Graphics for current part ******
!
part attributes  &
   part_name = .SCARA.BASE  &
   color = COLOR_R165G158B149
!
!----------------------------------- Link1 ------------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.ground
!
part create rigid_body name_and_position  &
   part_name = .SCARA.Link1  &
   adams_id = 3  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.Link1
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .SCARA.Link1.PSMAR  &
   adams_id = 13  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
marker attributes  &
   marker_name = .SCARA.Link1.PSMAR  &
   visibility = off
!
marker create  &
   marker_name = .SCARA.Link1.O1  &
   adams_id = 4  &
   location = 0.0, 0.0, -2.5E-02  &
   orientation = 0.0, 0.0, 0.0
!
marker create  &
   marker_name = .SCARA.Link1.O2  &
   adams_id = 5  &
   location = 0.2, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
marker create  &
   marker_name = .SCARA.Link1.MARKER_17  &
   adams_id = 17  &
   location = 0.0, 0.0, -2.5E-02  &
   orientation = 0.0, 0.0, 0.0
!
marker create  &
   marker_name = .SCARA.Link1.MARKER_20  &
   adams_id = 20  &
   location = 0.2, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
! ****** Graphics for current part ******
!
part attributes  &
   part_name = .SCARA.Link1  &
   color = COLOR_R212G171B032
!
!----------------------------------- Link2 ------------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.ground
!
part create rigid_body name_and_position  &
   part_name = .SCARA.Link2  &
   adams_id = 4  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.Link2
!
! ****** Markers for current part ******
!
marker create  &
   marker_name = .SCARA.Link2.PSMAR  &
   adams_id = 14  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
marker attributes  &
   marker_name = .SCARA.Link2.PSMAR  &
   visibility = off
!
marker create  &
   marker_name = .SCARA.Link2.cm  &
   adams_id = 15  &
   location = 0.2, -5.5363278464E-02, 1.0215672014E-02  &
   orientation = 0.0, 1.5102248077, 0.0
!
marker create  &
   marker_name = .SCARA.Link2.O2  &
   adams_id = 6  &
   location = 0.2, 0.0, 0.0  &
   orientation = 4.7123889804, 0.0, 0.0
!
marker create  &
   marker_name = .SCARA.Link2.MARKER_19  &
   adams_id = 19  &
   location = 0.2, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
marker create  &
   marker_name = .SCARA.Link2.P  &
   adams_id = 16  &
   location = 0.2, -0.1442, 2.5E-02  &
   orientation = 4.7123889804, 0.0, 0.0
!
part create rigid_body mass_properties  &
   part_name = .SCARA.Link2  &
   density = 2700.0
!
! ****** Graphics for current part ******
!
part attributes  &
   part_name = .SCARA.Link2  &
   color = COLOR_R136G089B086
!
! ****** Graphics from Parasolid file ******
!
file parasolid read  &
   file_name = "Controls_Plant_1.xmt_txt"  &
   model_name = .SCARA
!
geometry attributes  &
   geometry_name = .SCARA.BASE.SOLID1  &
   color = COLOR_R165G158B149
!
geometry attributes  &
   geometry_name = .SCARA.Link1.SOLID2  &
   color = COLOR_R212G171B032
!
geometry attributes  &
   geometry_name = .SCARA.Link2.SOLID3  &
   color = COLOR_R136G089B086
!
!----------------------------------- Joints -----------------------------------!
!
!
constraint create joint fixed  &
   joint_name = .SCARA.FIX  &
   adams_id = 1  &
   i_marker_name = .SCARA.ground.MARKER_1  &
   j_marker_name = .SCARA.BASE.MARKER_2
!
constraint attributes  &
   constraint_name = .SCARA.FIX  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .SCARA.JOINT_1  &
   adams_id = 2  &
   i_marker_name = .SCARA.BASE.O1  &
   j_marker_name = .SCARA.Link1.O1
!
constraint attributes  &
   constraint_name = .SCARA.JOINT_1  &
   name_visibility = off
!
constraint create joint revolute  &
   joint_name = .SCARA.JOINT_2  &
   adams_id = 3  &
   i_marker_name = .SCARA.Link1.O2  &
   j_marker_name = .SCARA.Link2.O2
!
constraint attributes  &
   constraint_name = .SCARA.JOINT_2  &
   name_visibility = off
!
!----------------------------------- Forces -----------------------------------!
!
!
force create direct single_component_force  &
   single_component_force_name = .SCARA.SFORCE_1  &
   adams_id = 1  &
   type_of_freedom = rotational  &
   i_marker_name = .SCARA.Link1.MARKER_17  &
   j_marker_name = .SCARA.BASE.MARKER_18  &
   action_only = off  &
   function = ""
!
force create direct single_component_force  &
   single_component_force_name = .SCARA.SFORCE_2  &
   adams_id = 2  &
   type_of_freedom = rotational  &
   i_marker_name = .SCARA.Link2.MARKER_19  &
   j_marker_name = .SCARA.Link1.MARKER_20  &
   action_only = off  &
   function = ""
!
!----------------------------- Simulation Scripts -----------------------------!
!
!
simulation script create  &
   sim_script_name = .SCARA.Last_Sim  &
   commands =   &
              "simulation single_run transient type=auto_select initial_static=no end_time=5.0 number_of_steps=50 model_name=.SCARA"
!
!-------------------------- Adams View UDE Instances --------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.ground
!
undo begin_block suppress = yes
!
ude create instance  &
   instance_name = .SCARA.Controls_Plant_1  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .SCARA.Model_Controls  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
ude create instance  &
   instance_name = .SCARA.JOINTSPACE  &
   definition_name = .controls.controls_plant  &
   location = 0.0, 0.0, 0.0  &
   orientation = 0.0, 0.0, 0.0
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.input_channels  &
   object_value =   &
      .SCARA.GAMMA1,  &
      .SCARA.GAMMA2
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.output_channels  &
   object_value =   &
      .SCARA.Px,  &
      .SCARA.Py,  &
      .SCARA.Q1,  &
      .SCARA.Q2,  &
      .SCARA.QP1,  &
      .SCARA.QP2
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.file_name  &
   string_value = "Controls_Plant_1"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.host  &
   string_value = "phs_pc"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .SCARA.Controls_Plant_1.include_mnf  &
   string_value = "no"
!
ude modify instance  &
   instance_name = .SCARA.Controls_Plant_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .SCARA.Model_Controls.input_channels  &
   object_value =   &
      .SCARA.GAMMA1,  &
      .SCARA.GAMMA2
!
variable modify  &
   variable_name = .SCARA.Model_Controls.output_channels  &
   object_value =   &
      .SCARA.Q1,  &
      .SCARA.Q2
!
variable modify  &
   variable_name = .SCARA.Model_Controls.file_name  &
   string_value = "Controls_Plant_1"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .SCARA.Model_Controls.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .SCARA.Model_Controls.host  &
   string_value = "ADMIN-PC"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .SCARA.Model_Controls.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .SCARA.Model_Controls.include_mnf  &
   string_value = "yes"
!
ude modify instance  &
   instance_name = .SCARA.Model_Controls
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.input_channels  &
   object_value =   &
      .SCARA.GAMMA1,  &
      .SCARA.GAMMA2
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.output_channels  &
   object_value =   &
      .SCARA.Px,  &
      .SCARA.Py,  &
      .SCARA.Q1,  &
      .SCARA.Q2,  &
      .SCARA.QP1,  &
      .SCARA.QP2
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.file_name  &
   string_value = "JOINTSPACE"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.solver_type  &
   string_value = "cplusplus"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.target  &
   string_value = "MATLAB"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.analysis_type  &
   string_value = "non_linear"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.analysis_init  &
   string_value = "no"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.analysis_init_str  &
   string_value = ""
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.user_lib  &
   string_value = ""
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.host  &
   string_value = "ADMIN-PC"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.dynamic_state  &
   string_value = "on"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.tcp_ip  &
   string_value = "off"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.output_rate  &
   integer_value = 1
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.realtime  &
   string_value = "off"
!
variable modify  &
   variable_name = .SCARA.JOINTSPACE.include_mnf  &
   string_value = "yes"
!
ude modify instance  &
   instance_name = .SCARA.JOINTSPACE
!
undo end_block
!
!------------------------------ Dynamic Graphics ------------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = .SCARA.ground
!
geometry create shape force  &
   force_name = .SCARA.SFORCE_1_force_graphic_1  &
   adams_id = 4  &
   force_element_name = .SCARA.SFORCE_1  &
   applied_at_marker_name = .SCARA.Link1.MARKER_17
!
geometry create shape force  &
   force_name = .SCARA.SFORCE_1_2_force_graphic_1  &
   adams_id = 5  &
   force_element_name = .SCARA.SFORCE_2  &
   applied_at_marker_name = .SCARA.Link2.MARKER_19
!
!---------------------------------- Accgrav -----------------------------------!
!
!
force create body gravitational  &
   gravity_field_name = gravity  &
   x_component_gravity = 0.0  &
   y_component_gravity = 0.0  &
   z_component_gravity = -9.80665
!
!----------------------------- Analysis settings ------------------------------!
!
!
!---------------------------- Adams View Variables ----------------------------!
!
!
variable create  &
   variable_name = .SCARA._model  &
   string_value = ".SCARA"
!
!---------------------------- Function definitions ----------------------------!
!
!
data_element modify variable  &
   variable_name = .SCARA.Q1  &
   function = "AZ( .SCARA.Link1.O1, .SCARA.BASE.O1)"
!
data_element modify variable  &
   variable_name = .SCARA.Q2  &
   function = "AZ( .SCARA.Link2.O2, .SCARA.Link1.O2)"
!
data_element modify variable  &
   variable_name = .SCARA.GAMMA1  &
   function = "0.00000000000000000e+000"
!
data_element modify variable  &
   variable_name = .SCARA.GAMMA2  &
   function = "0.00000000000000000e+000"
!
data_element modify variable  &
   variable_name = .SCARA.Px  &
   function = "DX( .SCARA.Link2.P, .SCARA.BASE.O1, .SCARA.BASE.O1)"
!
data_element modify variable  &
   variable_name = .SCARA.Py  &
   function = "DY( .SCARA.Link2.P, .SCARA.BASE.O1, .SCARA.BASE.O1)"
!
data_element modify variable  &
   variable_name = .SCARA.QP1  &
   function = "WZ( .SCARA.Link1.O1, .SCARA.BASE.O1, .SCARA.BASE.O1)"
!
data_element modify variable  &
   variable_name = .SCARA.QP2  &
   function = "WZ(.SCARA.Link2.O2, .SCARA.Link1.O2, .SCARA.Link1.O2)"
!
force modify direct single_component_force  &
   single_component_force_name = .SCARA.SFORCE_1  &
   function = "VARVAL(.SCARA.GAMMA1)"
!
force modify direct single_component_force  &
   single_component_force_name = .SCARA.SFORCE_2  &
   function = "VARVAL(.SCARA.GAMMA2)"
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .SCARA.Controls_Plant_1
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .SCARA.Model_Controls
!
!-------------------------- Adams View UDE Instance ---------------------------!
!
!
ude modify instance  &
   instance_name = .SCARA.JOINTSPACE
!
!--------------------------- Expression definitions ---------------------------!
!
!
defaults coordinate_system  &
   default_coordinate_system = ground
!
material modify  &
   material_name = .SCARA.steel  &
   density = (7801.0(kg/meter**3))  &
   youngs_modulus = (2.07E+11(Newton/meter**2))
!
geometry modify shape force  &
   force_name = .SCARA.SFORCE_1_force_graphic_1  &
   applied_at_marker_name = (.SCARA.SFORCE_1.i)
!
geometry modify shape force  &
   force_name = .SCARA.SFORCE_1_2_force_graphic_1  &
   applied_at_marker_name = (.SCARA.SFORCE_2.i)
!
model display  &
   model_name = SCARA
