#ifndef M_STRUCT_DEF_H
#define M_STRUCT_DEF_H

#include <cstdint>

#include <matrix/math.hpp>
#include <mathlib/math/Utilities.hpp>

#ifdef __cplusplus
struct __EXPORT estimator_aid_source1 {
#else
struct estimator_aid_source1 {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t time_last_fuse;
	uint32_t device_id;
	float observation;
	float observation_variance;
	float innovation;
	float innovation_filtered;
	float innovation_variance;
	float test_ratio;
	float test_ratio_filtered;
	uint8_t estimator_instance;
	bool innovation_rejected;
	bool fused;
	uint8_t _padding0[5]; // required for logger


#ifdef __cplusplus

#endif
};

#ifdef __cplusplus
struct __EXPORT estimator_aid_source2 {
#else
struct estimator_aid_source2 {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t time_last_fuse;
	uint32_t device_id;
	float observation[2];
	float observation_variance[2];
	float innovation[2];
	float innovation_filtered[2];
	float innovation_variance[2];
	float test_ratio[2];
	float test_ratio_filtered[2];
	uint8_t estimator_instance;
	bool innovation_rejected;
	bool fused;
	uint8_t _padding0[1]; // required for logger


#ifdef __cplusplus

#endif
};


#ifdef __cplusplus
struct __EXPORT estimator_aid_source3 {
#else
struct estimator_aid_source3 {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t time_last_fuse;
	uint32_t device_id;
	float observation[3];
	float observation_variance[3];
	float innovation[3];
	float innovation_filtered[3];
	float innovation_variance[3];
	float test_ratio[3];
	float test_ratio_filtered[3];
	uint8_t estimator_instance;
	bool innovation_rejected;
	bool fused;
	uint8_t _padding0[5]; // required for logger


#ifdef __cplusplus

#endif
};

#endif // !M_STRUCT_DEF_H
