#include "ros/ros.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>

namespace i2c_ros
{
	class I2C
	{
		public:
			I2C(int, int);
			virtual ~I2C();
			uint8_t readBytes(uint8_t* buff,uint8_t bufferSize);
			int writeData(uint8_t* data,uint8_t dataSize);
		private:
			int _i2caddr;
			int _i2cbus;
			void openfd();
			char busfile[64];
			int fd;
	};
}

