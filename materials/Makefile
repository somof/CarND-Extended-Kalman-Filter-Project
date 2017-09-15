all: L23

L6: L6_kalman_filter.cpp
	g++ -I../src/Eigen $< -o $@; ./$@

L12: L12_Laser_Measurements_main.cpp L12_kalman_filter.h L12_tracking.cpp L12_kalman_filter.cpp L12_measurement_package.h L12_tracking.h
	g++ -I../src/Eigen L12_Laser_Measurements_main.cpp L12_kalman_filter.h L12_tracking.cpp L12_kalman_filter.cpp L12_measurement_package.h L12_tracking.h -o $@;
	./$@

L18: L18_Jacobian_Matrix_main.cpp
	g++ -I../src/Eigen $< -o $@; ./$@

L23: L23_Evaluating_KF_Performance.cpp
	g++ -I../src/Eigen $< -o $@; ./$@
