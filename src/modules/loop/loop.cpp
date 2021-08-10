

#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/mytest.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/uORB.h>
#include <board_config.h>
extern "C" __EXPORT int loop_main(int argc, char *argv[]);

class Loop : public ModuleBase<Loop>, public px4::ScheduledWorkItem
{

public:
	Loop();
	~Loop()  override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	/* data */
	orb_advert_t _mavlink_log_pub = nullptr;
	perf_counter_t	_loop_perf;
	uORB::Subscription _sub_mytest{ORB_ID(mytest)};
	mytest_s mytest;
};





Loop::Loop():
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, "loop"))
{

}

Loop::~Loop()
{
	//orb_unsubscribe(_sub_rc_input);
	perf_free(_loop_perf);

}


bool Loop::init()
{
	ScheduleDelayed(10);
	return true;
}


void Loop::Run()
{

	// mavlink_log_info(&_mavlink_log_pub, "Loop RUN ");

	perf_begin(_loop_perf);
	// backup schedule as a watchdog timeout
	ScheduleDelayed(10);

		if (_sub_mytest.updated()) {
		_sub_mytest.copy(&mytest);
		mavlink_log_info(&_mavlink_log_pub,"Loop RUN %f %d", (double)mytest.x,mytest.mytest_id);
	 }


	perf_end(_loop_perf);
}

int Loop::task_spawn(int argc, char *argv[])
{
Loop *instance = new Loop();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		PX4_INFO("loop task  test loc");
		if (instance->init()) {
			PX4_INFO("loop task ok");
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int Loop::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Loop::print_status()
{

	return 0;
}


int Loop::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
my loop test
The provided functionality includes:

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("loop", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int loop_main(int argc, char *argv[])
{
	return Loop::main(argc, argv);
}
