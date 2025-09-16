1. Run Preprocessing Launch file(gaden_preproc_launch.py) from gaden/test_env/launch
### ros2 launch test_env gaden_preproc_launch.py scenario:=10x6_empty_room ###
The Results are saved in the Install/ directory -> test_env/scenario/10x6_empty_room and test_env/scenario/10x6_empty_room/wind_simulations
2. Run Simulation Launch file(gaden_sim_launch.py) from gaden/test_env/launch
### ros2 launch test_env gaden_sim_launch.py ###
The Results are saved in the Install/ directory -> test_env/scenario/10x6_empty_room/gas_simulations/dynamic/
3. Run the main Launch file from gaden_simulation_p
### ros2 launch gaden_simulation_p APF_1_1.py use_sim_time:=true ###
