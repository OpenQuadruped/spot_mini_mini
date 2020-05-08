#include "mini_ros/minitaur.hpp"

namespace mini
{

	// Minitaur Constructor
	Minitaur::Minitaur()
	{
		cmd.velocity = 0.0;
		cmd.rate = 0.0;
		cmd.motion = Stop;
		movement = FB;
	}

	void Minitaur::update_command(const double & v, const double & w)
	{
		if (!almost_equal(cmd.velocity, v) or !almost_equal(cmd.rate, w))
		{
			cmd.velocity = v;
			cmd.rate = w;

			if (almost_equal(cmd.velocity, 0.0) and almost_equal(cmd.rate, 0.0))
			{
				cmd.motion = Stop;
				cmd.velocity = 0.0;
				cmd.rate = 0.0;
			} else
			{
				// W,S (Up/Down) keys for FWD/BWD, A,D (Left/Right) keys for CCW, CW
				if (movement == FB)
				{
					if (cmd.velocity > 0.0 and !almost_equal(cmd.velocity, 0.0) and almost_equal(cmd.rate, 0.0))
					{
						cmd.motion = Forward;
					} else if (cmd.velocity < 0.0 and !almost_equal(cmd.velocity, 0.0) and almost_equal(cmd.rate, 0.0))
					{
						cmd.motion = Backward;
					} else if (cmd.rate > 0.0 and !almost_equal(cmd.rate, 0.0) and almost_equal(cmd.velocity, 0.0))
					{
						cmd.motion = CCW;
					} else if (cmd.rate < 0.0 and !almost_equal(cmd.rate, 0.0) and almost_equal(cmd.velocity, 0.0))
					{
						cmd.motion = CW;
					} 
				} else
				// W,S (Up/Down) keys for NOTHING, A,D (Left/Right) keys for Left/Right
				{
					if (cmd.rate > 0.0 and !almost_equal(cmd.rate, 0.0))
					{
						cmd.motion = Right;
					} else if (cmd.rate < 0.0 and !almost_equal(cmd.rate, 0.0))
					{
						cmd.motion = Left;
					} 
				}
			}
		}
		
	}

	void Minitaur::switch_movement()
	{
		if (!almost_equal(cmd.velocity, 0.0) and !almost_equal(cmd.rate, 0.0))
		{
			ROS_WARN("MAKE SURE BOTH LINEAR [%.2f] AND ANGULAR VELOCITY [%.2f] ARE AT 0.0 BEFORE SWITCHING!", cmd.velocity, cmd.rate);

			ROS_WARN("STOPPING ROBOT...");

			cmd.velocity = 0.0;
			cmd.rate = 0.0;
			cmd.motion = Stop;
		} else
		{
			if (movement == FB)
			{
				ROS_INFO("SWITCHING TO LEFT/RIGHT MOTION.");

				movement = LR;
				cmd.motion = Stop;
			} else
			{
				ROS_INFO("SWITCHING TO FORWARD/BACKWARD MOTION.");

				movement = FB;
				cmd.motion = Stop;
			}
		}
	}

	MiniCommand Minitaur::return_command()
	{
		return cmd;
	}
}