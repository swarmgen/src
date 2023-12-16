# Tools for paritioning, analysis, and data processing

During the project, we created several tools and scripts that slice the mission expressed as Dcc to obtaine the sub-missions. It can also assist our data analysis and processing tasks. The below list gives brief descriptions about the different tools.   
**Note that correct I/O paths should be set** before using tools (current paths are removed as they include author's name).

- Partitioning the swarm mission (Dcc values).
  - [01_partitioning](01_partitioning) 
- Analysis trajectories converted from simulation to physical experiment (lab environment)
  - [02_analysis_traj_physical_exp](02_analysis_traj_physical_exp)
- Coordinates analysis for comparison between trajectories (swarm flights)
  - [03_analysis_coord](03_analysis_coord)
- Data visualization for Dcc (raw from the simulation)
  - [04_plotting_dcc](04_plotting_dcc)
- Data visualization for the result after applying all rules
  - [05_plotting_aggregated_result](05_plotting_aggregated_result)
- Data visualization for sub-missions
  - [06_plotting_individual_sub_missions](06_plotting_individual_sub_missions)
- Piecewise linear regression
  - [07_piecewise_linear](07_piecewise_linear)
- Analyzing and testing each step for partitioning
  - [08_partitioning_steps](08_partitioning_steps)
- Library for the other tools
  - [99_lib](99_lib)
