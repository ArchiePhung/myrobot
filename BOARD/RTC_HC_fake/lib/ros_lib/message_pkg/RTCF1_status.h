#ifndef _ROS_message_pkg_RTCF1_status_h
#define _ROS_message_pkg_RTCF1_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace message_pkg
{
	class RTCF1_status : public ros::Msg
	{
		public:
			typedef uint8_t _status_type;
			_status_type status;

		RTCF1_status():
			status(0)
		{
		}

		virtual int serialize(unsigned char *outbuffer) const
		{
			int offset = 0;

			union {
				uint8_t real;
				uint8_t base;
			} u_status;
			u_status.real = this->status;
			*(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->status);

			return offset;
		}

		virtual int deserialize(unsigned char *inbuffer)
		{
			int offset = 0;

			union {
				uint8_t real;
				uint8_t base;
			} u_status;
			u_status.base = 0;
			u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->status = u_status.real;
			offset += sizeof(this->status);

			return offset;
		}

		const char * getType(){ return "message_pkg/RTCF1_status"; };
		const char * getMD5(){ return "284aa12dd9e9e760802ac9f38036ea5e"; };

	};

}
#endif