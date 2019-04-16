/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Airmind nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file us_100.cpp
 * @author Yifei Gu <guyifei10162@163.com>
 * 
 * Driver for the US-100 sonar range finders.
 */

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#include <termios.h>

#ifndef __PX4_QURT
#include <poll.h>
#endif


#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_cli.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <perf/perf_counter.h>

#include <board_config.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

// define baudrate
#define US100_BAUDRATE			B9600

// designated serial port on Pixhawk (SERIAL 4)
#define US100_DEFAULT_PORT		"/dev/ttyS6"

// Device limits
#define US100_MIN_DISTANCE		(0.02f)
#define US100_MAX_DISTANCE		(4.50f)

// normal conversion wait time
#define US100_CONVERSION_INTERVAL	(30*1000UL)	/* 30ms */

// 测量间隔
#define US100_MEASURE_INTERVAL_SET	(100*1000UL)	/* 100ms */
#define US100_MEASURE_INTERVAL		((US100_MEASURE_INTERVAL_SET>US100_CONVERSION_INTERVAL)?\
		(US100_MEASURE_INTERVAL_SET - US100_CONVERSION_INTERVAL):(0))

// command
#define US100_TRIG			"\x55"

class US_100 : public ModuleBase<US_100>
{
public:
	US_100(const char *port = US100_DEFAULT_PORT, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	// Virtual destructor
	virtual ~US_100();

	virtual int  init();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static US_100 *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	int print_status() override;

private:

	char 				_port[20];
	uint8_t                   	_rotation;
	float				_min_distance;
	float				_max_distance;
	ringbuffer::RingBuffer		*_reports;
	int				_serial_fd;
	uint8_t				_linebuf[4];


	int				_distance_mm;
	int				_orb_class_instance;
	
	orb_advert_t			_distance_sensor_topic;

	perf_counter_t			_sample_perf;
	perf_counter_t			_measure_interval;
	perf_counter_t			_comms_errors_rx;
	perf_counter_t			_comms_errors_tx;

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	int				start();

	/**
	 * This is an abstraction for the poll on serial used.
	 *
	 * @param buf: pointer to read buffer
	 * @param buf_length: size of read buffer
	 * @param timeout: timeout in ms
	 * @return: 0 for nothing read, or poll timed out
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int 			pollOrRead(uint8_t *buf, size_t buf_length, int timeout);

	/**
	 * Set the min and max distance thresholds.
	*/
	inline void		set_minimum_distance(float min);
	inline void		set_maximum_distance(float max);
	inline float		get_minimum_distance();
	inline float		get_maximum_distance();


	int				measure();
	int				collect();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int us_100_main(int argc, char *argv[]);


/**
 * Method : Constructor
 * 类方法 :  构造函数
 *
 * @note This method initializes the class variables
*/
US_100::US_100(const char *port, uint8_t rotation) :
	_rotation(rotation),
	_min_distance(US100_MIN_DISTANCE),
	_max_distance(US100_MAX_DISTANCE),
	_reports(nullptr),
	_serial_fd(-1),
	_distance_mm(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),

	_sample_perf(perf_alloc(PC_ELAPSED, "us_100_read")),
	_measure_interval(perf_alloc(PC_INTERVAL, "us_100_measure_interval")),
	_comms_errors_rx(perf_alloc(PC_COUNT, "us_100_com_err_rx")),
	_comms_errors_tx(perf_alloc(PC_COUNT, "us_100_com_err_tx"))
{
	/* store port name */
	// 保存端口名
	strncpy(_port, port, sizeof(_port));

	/* enforce null termination */
	// 强制在末尾添上'\0'
	_port[sizeof(_port) - 1] = '\0';
}

// Destructor
// 析构函数
US_100::~US_100()
{
	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	perf_free(_sample_perf);
	perf_free(_measure_interval);
	perf_free(_comms_errors_rx);
	perf_free(_comms_errors_tx);
}

/**
 * Method : init()
 * 类方法 :  初始化
 *
 * This method setup the general driver for a range finder sensor.
*/
int
US_100::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* allocate basic report buffers */
		// 分配报告缓冲区
		_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

		if (_reports == nullptr) {
			PX4_ERR("alloc failed");
			ret = -1;
			break;
		}

	} while (0);
	return ret;
}

int
US_100::start()
{
	int ret = OK;

	PX4_INFO("driver starting...");

	/* fds initialized? */
	// 如果文件描述符未初始化
	if (_serial_fd < 0) {
		/* open fd */
		// 打开端口
		// O_RDWR: 读写模式
		// O_NOCTTY: 如果路径名指向终端设备，不要把这个设备用作控制终端。
		// O_NONBLOCK: 如果路径名指向FIFO/块文件/字符文件，则把文件的打开和后继I/O设置为非阻塞模式
		_serial_fd = open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (_serial_fd < 0) {
			PX4_ERR("US-100: failed to open serial port: %s err: %d", _port, errno);
			ret = -ENODEV;
			return ret;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		// 将_serial_fd的状态放入uart_config
		tcgetattr(_serial_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		// 清除ONLCR标志位
		// ONLCR: (XSI) 将输出中的新行符映射为回车-换行。
		uart_config.c_oflag &= ~(ONLCR);

		/* no parity, one stop bit */
		// 清除CSTOPB和PARENB标志位（设为1停止位，无校验）
		// CSTOPB: 设置两个停止位，而不是一个。
		// PARENB: 允许输出产生奇偶信息以及输入的奇偶校验。
		// CRTSCTS 使用RTS/CTS流控制
		uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

		unsigned speed = US100_BAUDRATE;

		/* set baud rate */
		// 设置输入波特率
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -EPERM;
		}
		// 设置输出波特率
		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
			ret = -EPERM;
		}
		//应用串口配置
		if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -EPERM;
		}
	}

	/* reset the report ring and state machine */
	_reports->flush();

	return ret;
}

inline void
US_100::set_minimum_distance(float min)
{
	_min_distance = min;
}

inline void
US_100::set_maximum_distance(float max)
{
	_max_distance = max;
}

inline float
US_100::get_minimum_distance()
{
	return _min_distance;
}

inline float
US_100::get_maximum_distance()
{
	return _max_distance;
}

/*
 * Method: measure()
 * 类方法:  开始测量
 *
*/
int
US_100::measure()
{
	int ret = OK;

	perf_count(_measure_interval);

	// 清空缓冲区
	while(pollOrRead(_linebuf, sizeof(_linebuf), 0) > 0);;

	//发送测距指令
	size_t written = write(_serial_fd, US100_TRIG, strlen(US100_TRIG));
	if (strlen(US100_TRIG) != written)
	{
		perf_count(_comms_errors_tx);
		//DEVICE_LOG("error reading from sensor: %d", ret);
		ret = -EIO;
	}

	return ret;
}


/*
 * Method: collect()
 * 类方法:  收集数据
 *
 * This method reads data from serial UART and places it into a buffer
*/
int
US_100::collect()
{
	int ret = -EIO;

	int bytes_read = 0;
	bool valid = false;

	perf_begin(_sample_perf);

	/* read from the sensor (uart buffer) */
	bytes_read = pollOrRead(_linebuf, sizeof(_linebuf), 50);

	// 解析数据
	if (bytes_read <= 0)
	{
		// 传感器没有连接，则有可能没有返回数据
		ret = -EIO;
		PX4_DEBUG("read err: %d \n", ret);
		perf_count(_comms_errors_rx);
		perf_end(_sample_perf);
		return ret;
	}
	else if (bytes_read >= 2)
	{
		_distance_mm = uint8_t(_linebuf[0])*256 + uint8_t(_linebuf[1]);

		if(_distance_mm > get_maximum_distance()*1000 || _distance_mm < get_minimum_distance()*1000)
		{
			valid = false;

			// TODO: 决定超出范围时是否发布数据（EKF2中应该会判断数据是否合法）
			ret = OK;
			perf_end(_sample_perf);
			return ret;
		}
		else
		{
			valid = true;
		}
	}
	else {
		ret = -EAGAIN;
		perf_count(_comms_errors_rx);
		perf_end(_sample_perf);
		return ret;
	}

	struct distance_sensor_s report;

	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = _rotation;

	report.current_distance = _distance_mm / 1000.0f;

	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.variance = 0.0f;

	// 0 = invalid signal, -1 = unknown signal quality.
	// 如果为0且在空中，则ekf2不更新range finder数据
	report.signal_quality = (valid) ? (-1) : (0);

	/* TODO: set proper ID */
	report.id = 0;

	/* publish it */
	// 发布
	orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report, &_orb_class_instance,
				 ORB_PRIO_DEFAULT);

	_reports->force(&report);


	ret = OK;

	perf_end(_sample_perf);

	return ret;

}

int US_100::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	const int max_timeout = 100;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;
	
	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

	if (ret > 0) {
		/* if we have new data from US-100, go handle it */
		if (fds[0].revents & POLLIN) {
			/*
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If we have all requested data available, read it without waiting.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			const unsigned character_count = 2; // minimum bytes that we want to read

			const unsigned sleeptime = character_count * 1000000 / (9600 / 10);

			int err = 0;
			int bytes_available = 0;
			err = ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

			if (err != 0 || bytes_available < (int)character_count) {
				px4_usleep(sleeptime);
			}
			ret = read(_serial_fd, buf, buf_length);

		} else {
			ret = -1;
		}
	}
	
	return ret;

}

void
US_100::run()
{
	int err;
	err = init();
	if (err == OK) {
		err = start();
	}
	if(err == OK) {
		int ret;
		/* loop handling received serial bytes and also configuring in between */
		while (!should_exit()) {
			if(OK != (ret=measure())) {
				//PX4_WARN("measure error (%i), retry in 1s", ret);
				px4_usleep(1000000UL);
				continue;
			}
			else {
				px4_usleep(US100_CONVERSION_INTERVAL);
			}
			if (OK != (ret=collect())) {
				//PX4_WARN("collect error (%i), retry in 1s", ret);
				px4_usleep(1000000UL);
			}
			else
			{
				px4_usleep(US100_MEASURE_INTERVAL);
			}
			
		}
	}
	else {
		PX4_ERR("driver failed!");
	}
	PX4_INFO("exiting...");

	if (_serial_fd >= 0) {
		close(_serial_fd);
		_serial_fd = -1;
	}

	orb_unadvertise(_distance_sensor_topic);

}

int
US_100::print_status()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_interval);
	perf_print_counter(_comms_errors_tx);
	perf_print_counter(_comms_errors_rx);
	_reports->print_info("report queue");
	return 0;
}

int
US_100::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int US_100::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the US-100 sonar range finders.

Author: Yifei Gu <guyifei10162@163.com>

### Examples
TODO:
$ us_100 start
$ us_100 start -d /dev/ttyS2
$ us_100 status
$ us_100 stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("us_100", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', US100_DEFAULT_PORT, "<file:dev>", "US-100 device", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
int US_100::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("us_100", SCHED_DEFAULT,
				   SCHED_PRIORITY_SENSOR_HUB, 1600,
				   (px4_main_t)&run_trampoline, (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

US_100 *US_100::instantiate(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_name = US100_DEFAULT_PORT;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_name = myoptarg;
			break;
		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	US_100 *us100;
	us100 = new US_100(device_name, rotation);

	return us100;
}

int
us_100_main(int argc, char *argv[])
{
	return US_100::main(argc, argv);
}
