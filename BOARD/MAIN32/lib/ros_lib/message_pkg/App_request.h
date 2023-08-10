#ifndef _ROS_message_pkg_App_request_h
#define _ROS_message_pkg_App_request_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace message_pkg
{
	class App_request : public ros::Msg
	{
		public:
			typedef uint8_t _mode_type;
			_mode_type mode;	
			typedef uint8_t _action_type;
			_action_type action;
			typedef float _vel_type;
			_vel_type vel;
			

		App_request():
			mode(0),
			action(0),
			vel(0)
		{
		}

		virtual int serialize(unsigned char *outbuffer) const
		{
			int offset = 0;

			union {
				uint8_t real;
				uint8_t base;
			} u_mode;
			u_mode.real = this->mode;
			*(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->mode);
			
			union {
				uint8_t real;
				uint8_t base;
			} u_action;
			u_action.real = this->action;
			*(outbuffer + offset + 0) = (u_action.base >> (8 * 0)) & 0xFF;
			offset += sizeof(this->action);

			union {
				float real;
				uint32_t base;
			} u_vel;
			u_vel.real = this->vel;
			*(outbuffer + offset + 0) = (u_vel.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_vel.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_vel.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_vel.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->vel);

			return offset;
		}

		virtual int deserialize(unsigned char *inbuffer)
		{
			int offset = 0;

			union {
				uint8_t real;
				uint8_t base;
			} u_mode;
			u_mode.base = 0;
			u_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->mode = u_mode.real;
			offset += sizeof(this->mode);

			union {
				uint8_t real;
				uint8_t base;
			} u_action;
			u_action.base = 0;
			u_action.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
			this->action = u_action.real;
			offset += sizeof(this->action);

			union {
				float real;
				uint32_t base;
			} u_vel;
			u_vel.base = 0;
			u_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
			this->vel = u_vel.real;
			offset += sizeof(this->vel);

			return offset;
		}

		const char * getType(){ return "message_pkg/App_request"; };
		const char * getMD5(){ return "f6fb3554f499951649fc2f38c3f0f6fa"; };

	};

}
#endif