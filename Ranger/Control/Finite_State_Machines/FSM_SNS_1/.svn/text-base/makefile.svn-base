


fsm_project: read_global_sensors.o main.o fsm.o top_fsm.o ankle_fsm.o ankle_fsm_sensors.o hip_fsm.o hip_fsm_sensors.o top_fsm_sensors.o
	g++ read_global_sensors.o main.o fsm.o top_fsm.o ankle_fsm.o ankle_fsm_sensors.o hip_fsm.o hip_fsm_sensors.o top_fsm_sensors.o -o fsm_project
fsm.o: fsm.h fsm.cpp
	g++ -c fsm.cpp
read_global_sensors.o: read_global_sensors.h global_sensors.h read_global_sensors.cpp
	g++ -c read_global_sensors.cpp
hip_fsm.o: global_sensors.h global_params.h fsm.h hip_fsm.h hip_fsm.cpp
	g++ -c hip_fsm.cpp
hip_fsm_sensors.o: global_sensors.h hip_fsm_sensors.h hip_fsm_sensors.cpp
	g++ -c hip_fsm_sensors.cpp
ankle_fsm.o: global_sensors.h global_params.h fsm.h ankle_fsm.h ankle_fsm.cpp
	g++ -c ankle_fsm.cpp
ankle_fsm_sensors.o: global_sensors.h ankle_fsm_sensors.h ankle_fsm_sensors.cpp
	g++ -c ankle_fsm_sensors.cpp
top_fsm.o: global_sensors.h global_params.h fsm.h top_fsm.h top_fsm_sensors.h top_fsm.cpp
	g++ -c top_fsm.cpp
top_fsm_sensors.o: global_sensors.h top_fsm_sensors.h top_fsm_sensors.cpp
	g++ -c top_fsm_sensors.cpp
main.o: main.cpp read_global_sensors.h fsm.h hip_fsm.h ankle_fsm.h top_fsm.h
	g++ -c main.cpp
