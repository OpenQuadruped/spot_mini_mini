#include "mini_ros/spot.hpp"

namespace spot
{

	// Spot Constructor
	Spot::Spot()
	{
		cmd.x_velocity = 0.0;
		cmd.y_velocity = 0.0;
		cmd.rate = 0.0;
        cmd.roll = 0.0;
        cmd.pitch = 0.0;
        cmd.yaw = 0.0;
        cmd.z = 0.0;
		cmd.motion = Stop;
		cmd.movement = Viewing;
	}

	void Spot::update_command(const double & vx, const double & vy, const double & z,
							  const double & w, const double & wx, const double & wy)
	{
		// If Command is nearly zero, just give zero
		if (almost_equal(vx, 0.0) and almost_equal(vy, 0.0) and almost_equal(z, 0.0) and almost_equal(w, 0.0))
		{
			cmd.motion = Stop;
			cmd.x_velocity = 0.0;
			cmd.y_velocity = 0.0;
			cmd.rate = 0.0;
			cmd.roll = 0.0;
	        cmd.pitch = 0.0;
	        cmd.yaw = 0.0;
	        cmd.z = 0.0;
	        cmd.faster = 0.0;
	        cmd.slower = 0.0;
		} else
		{
			cmd.motion = Go;
			if (cmd.movement == Stepping)
			{
				// Stepping Mode, use commands as vx, vy, rate, Z
				cmd.x_velocity = vx;
				cmd.y_velocity = vy;
				cmd.rate = w;
				cmd.z = z;
				cmd.roll = 0.0;
		        cmd.pitch = 0.0;
		        cmd.yaw = 0.0;
		        // change clearance height from +- 0-2 * scaling
		        cmd.faster = 1.0 - wx;
		        cmd.slower = -(1.0 - wy);
			} else
			{
				// Viewing Mode, use commands as RPY, Z
				cmd.x_velocity = 0.0;
				cmd.y_velocity = 0.0;
				cmd.rate = 0.0;
				cmd.roll = vy;
		        cmd.pitch = vx;
		        cmd.yaw = w;
		        cmd.z = z;
		        cmd.faster = 0.0;
		        cmd.slower = 0.0;
			}
		}
		
	}

	void Spot::switch_movement()
	{
		if (!almost_equal(cmd.x_velocity, 0.0) and !almost_equal(cmd.y_velocity, 0.0) and !almost_equal(cmd.rate, 0.0))
		{
			ROS_WARN("MAKE SURE BOTH LINEAR [%.2f, %.2f] AND ANGULAR VELOCITY [%.2f] ARE AT 0.0 BEFORE SWITCHING!", cmd.x_velocity, cmd.y_velocity, cmd.rate);

			ROS_WARN("STOPPING ROBOT...");

			cmd.motion = Stop;
			cmd.x_velocity = 0.0;
			cmd.y_velocity = 0.0;
			cmd.rate = 0.0;
			cmd.roll = 0.0;
	        cmd.pitch = 0.0;
	        cmd.yaw = 0.0;
	        cmd.z = 0.0;
	        cmd.faster = 0.0;
	        cmd.slower = 0.0;
		} else
		{
			cmd.x_velocity = 0.0;
			cmd.y_velocity = 0.0;
			cmd.rate = 0.0;
			cmd.roll = 0.0;
	        cmd.pitch = 0.0;
	        cmd.yaw = 0.0;
	        cmd.z = 0.0;
	        cmd.faster = 0.0;
	        cmd.slower = 0.0;
			if (cmd.movement == Viewing)
			{
				ROS_INFO("SWITCHING TO STEPPING MOTION, COMMANDS NOW MAPPED TO VX|VY|W|Z.");

				cmd.movement = Stepping;
				cmd.motion = Stop;
			} else
			{
				ROS_INFO("SWITCHING TO VIEWING MOTION, COMMANDS NOW MAPPED TO R|P|Y|Z.");

				cmd.movement = Viewing;
				cmd.motion = Stop;
			}
		}
	}

	SpotCommand Spot::return_command()
	{
		return cmd;
	}
}