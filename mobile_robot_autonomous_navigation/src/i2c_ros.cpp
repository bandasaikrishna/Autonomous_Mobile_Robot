#include <mobile_robot_autonomous_navigation/i2c_ros.h>

namespace i2c_ros
{
	I2C::I2C(int bus, int address) {
		_i2cbus = bus;
		_i2caddr = address;
		snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
		openfd();
	}

	I2C::~I2C() {
		close(fd);
	}

	uint8_t I2C::readBytes(uint8_t* buff,uint8_t bufferSize) {
		if (fd != -1) {
			//uint8_t buff[bufferSize];
			if (read(fd, buff, bufferSize) != bufferSize) {
					ROS_ERROR("Could not read from I2C slave 0x%x, [read_byte():read %d]", _i2caddr,  errno);
					return (0);
			}
			else
{
//ROS_INFO("I2C buff: %s",buff);
				return 1;
}
		} else {
			ROS_ERROR("Device File not available. Aborting read");
			return (0);
		}
	}

	int I2C::writeData(uint8_t* data, uint8_t dataSize) {
		if (fd != -1) {
			int result = write(fd, data, dataSize);
			if (result != dataSize) {
				ROS_ERROR("%s. Failed to write to I2C Slave 0x%x @ [write_byte():write %d]", strerror(errno), _i2caddr, errno);
				return result;
			} else {
				//ROS_INFO("Wrote to I2C Slave 0x%x", _i2caddr);
				return result;
			}
		} else {
			ROS_ERROR("Device File not available. Aborting write");
			return (-1);
		}
		return 0;
	}

	void I2C::openfd() {
		if ((fd = open(busfile, O_RDWR)) < 0) {
			ROS_ERROR("Couldn't open I2C Bus %d [openfd():open %d]", _i2cbus, errno);
		}
		if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0) {
			ROS_ERROR("I2C slave %d failed [openfd():ioctl %d]", _i2caddr, errno);
		}
	}
}

