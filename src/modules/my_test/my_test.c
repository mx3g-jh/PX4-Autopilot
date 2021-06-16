//#include <px4_config.h>
#include <px4_platform_common/px4_config.h>
//#include <px4_tasks.h>
#include <px4_platform_common/tasks.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/my_test_public.h>
#include <termios.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <px4_defines.h>
#include <px4_time.h>

#include <float.h>
#include "my_test.h"
 //#include <px4_posix.h>
 #include <px4_platform_common/posix.h>
//#include <lib/ecl/geo/geo.h>
 #include <mathlib/mathlib.h>
// #include <matrix/math.hpp>
 #include <uORB/topics/sensor_combined.h>
// #include <drivers/drv_tone_alarm.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/distance_sensor.h>
#include <lib/systemlib/mavlink_log.h>

#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/mavlink_log.h>
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int px4_test_public;				/**< Handle of daemon task / thread */


__EXPORT int my_test_main(int argc, char *argv[]);

int my_test_thread_main(int argc, char *argv[]);
//bool detect_hold_still(float x , float y , float z, bool lenient_still_position);
static void usage(const char *reason);

static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }
    warnx("usage: my_test {start|stop|status} [-p <additional params>]\n\n");
}

int my_test_main(int argc, char *argv[])
{
    if (argc < 2) {
            usage("missing command");
            return 1;
        }

    if (!strcmp(argv[1], "start")) {

            if (thread_running) {
                warnx("daemon already running\n");
                /* this is not an error */
                return 0;
            }

            thread_should_exit = false;//定义一个守护进程
                   px4_test_public = px4_task_spawn_cmd(
                                    "my_test_app",
                                     SCHED_DEFAULT,
                                     SCHED_PRIORITY_DEFAULT,//调度优先级
                                     2000,//堆栈分配大小
                                     my_test_thread_main,
                                     (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                    return 0;
                }

             if (!strcmp(argv[1], "stop")) {
                 thread_should_exit = true;
                 return 0;
                 }

             if (!strcmp(argv[1], "status")) {
                    if (thread_running) {
                      warnx("\trunning\n");

                  } else {
                      warnx("\tnot started\n");
                  }
                    return 0;
                 }

             usage("unrecognized command");
             return 1;

   }

int my_test_thread_main(int argc, char *argv[])
{

   int ds_sub = orb_subscribe(ORB_ID(distance_sensor));
   struct distance_sensor_s ds_sen;
    memset(&ds_sen, 0, sizeof(ds_sen));


PX4_INFO(" begin!");



/********************************参数begin**********************************/

/*********************************参数end*************************************/

thread_running = true;
while (!thread_should_exit) {
/******************************************main loop begin*******************************************************/

PX4_INFO("Loop begin!");
px4_pollfd_struct_t fds[] = {
            { .fd = ds_sub, .events = POLLIN },

            };
             int error_counter = 0;
             int poll_ret = px4_poll(fds, 1, 1000);
             if (poll_ret == 0) {
                 PX4_ERR("Got no data within a second");
             } else if (poll_ret < 0) {
                 if (error_counter < 10 || error_counter % 50 == 0) {
                     PX4_ERR("ERROR return value from poll(): %d", poll_ret);
                 }
                 error_counter++;
             } else {
                    if (fds[0].revents & POLLIN)
                    {

                    orb_copy(ORB_ID(distance_sensor), ds_sub, &ds_sen);
                    PX4_INFO("\ndistance:\t%f ID：%d  RCS: %d covariance : %f\n", (double)ds_sen.current_distance,ds_sen.target_id,ds_sen.rcs,(double)ds_sen.covariance);






                   }

             }




 /*********************************************main loop end*******************************************************/
}

   // orb_unsubscribe(rc_sub_fd);
  //  orb_unsubscribe(rc_channel_fd);
    warnx("exiting.\n");
    thread_running = false;
    return 0;
}



