
#ifndef TRACKING_H_
#define TRACKING_H_

#include "L12_measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "L12_kalman_filter.h"

class Tracking {
public:
	Tracking();
	virtual ~Tracking();
	void ProcessMeasurement(const MeasurementPackage &measurement_pack);
	KalmanFilter kf_;

private:
	bool is_initialized_;
	int64_t previous_timestamp_;

	//acceleration noise components
	float noise_ax;
	float noise_ay;

};

#endif /* FUSION_KF_H_ */
