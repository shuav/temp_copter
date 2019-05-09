#include "Copter.h"


/*
void Copter::ModeZigZag::ModeZigZag(const AP_Proximity& proximity)
{
	_proximity(proximity)
	fly_direction=1;
}
*/

/***********************************************************************************************************************
*函数原型：bool Copter::ZigZag::init(bool ignore_checks)
*函数功能：AB点函数初始化
*修改日期：2018-9-10
*修改作者：cihang_uav
*备注信息：ZigZag_init - initialise stabilize controller
*************************************************************************************************************************/
bool Copter::ModeZigZag::init(bool ignore_checks)
{
	//测试用，风险，当心
	zigzag_mode = Zigzag_Auto;
	 return true;
/*
    if (!(motors->armed()) ) //没解锁就会立即退出初始化的过程
    {
        return false;
    }

     if (copter.position_ok()  || ignore_checks)  //位置ok，并且ignore_checks=1表示没有解锁，
     {

            //初始化航点状态------initialise waypoint state
        	zigzag_change_yaw = false;
            if(zigzag_waypoint_state.b_hasbeen_defined || zigzag_waypoint_state.bp_mode != Zigzag_None) //b点信息定义，没有断点
            {

            	//获得当前偏航数据----get_bearing_cd(_APoint, _BPoint) * 0.01f;
            	zigzag_bearing = get_bearing_cd(zigzag_waypoint_state.a_pos, zigzag_waypoint_state.b_pos); //获得当前航向

            	zigzag_mode = Zigzag_Auto;  //1
            	zigzag_auto_complete_state = (zigzag_waypoint_state.bp_mode != Zigzag_None);

            	zigzag_waypoint_state.flag = zigzag_waypoint_state.flag << 1;
            	zigzag_waypoint_state.flag += 1;

            	AP_Notify::flags.zigzag_record_mode=4;
            	if(AP_Notify::flags.zigzag_record_mode==4)
            	{
            		gcs().send_text(MAV_SEVERITY_WARNING,"Zigzag auto"); //发送自动信息
            	}


    			auto_yaw.set_mode(AUTO_YAW_HOLD); //设定自动偏航保持

            	wp_nav->wp_and_spline_init();
            	wp_nav->set_wp_destination(copter.current_loc); //当前的高度设置成目标高度
            	wp_nav->set_fast_waypoint(false);

            }
            else
            {
            	AP_Notify::flags.zigzag_record_mode=0;
            	gcs().send_text(MAV_SEVERITY_WARNING,"Zigzag manual ignore_checks"); //发送自动信息
            	zigzag_mode = Zigzag_Manual;
            	zigzag_auto_complete_state = false;

#if 1
            //初始化目标----initialize's loiter position and velocity on xy-axes from current pos and velocity
            loiter_nav->init_target();


            if (!pos_control->is_active_z())
            {

                pos_control->set_alt_target_to_current_alt();
                pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
            }
#else

                copter.mode_poshold.init(true);
#endif
            }

    		switch(zigzag_waypoint_state.direct) //copter.zigzag_waypoint_state.direct=0,默认是0
    		{

    			case 1:
    				zigzag_rc_state = RC_LEFT;  //在最左边
    				break;
    			case -1:
    			    zigzag_rc_state = RC_RIGHT; //在最右边
    				break;
    			case 0:
    			default:
    				zigzag_rc_state = RC_MID;  //默认在中间
    				break;
    		 }
             return true;
       }
        else
        {
            return false;
        }
        */
}

/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::run()
*函数功能：AB点函数运行
*修改日期：2018-9-10
*修改作者：cihang_uav
*备注信息：ZigZag_run - runs the main stabilize controller should be called at 100hz or more
*************************************************************************************************************************/
int next_temp=0;
extern float distant_temp;
extern Vector3f destination_temp;
Vector3f next_printf;//测试用

int state_zigzag=0;//测试用，zigzag状态变量


void Copter::ModeZigZag::run()
{
	//先关掉，切入自动模式，风险，当心！！！
		zigzag_mode=Zigzag_Auto;


	//如果没有自动解锁，电机没有使能，设置油门值为零，并且立即退出------ if not auto armed or motors not enabled set throttle to zero and exit immediately

 /*
	if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete)
		    {
		#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
		        // call attitude controller
		        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);//get_smoothing_gain()
		        attitude_control->set_throttle_out(0, false, g.throttle_filt);
		#else
		        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
		        loiter_nav->init_target();
		        attitude_control->reset_rate_controller_I_terms();
		        attitude_control->set_yaw_target_to_current_heading();
		        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero

		        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);
		        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), 0);
		        pos_control->update_z_controller();


		#endif
		        return;
		    }

	*/



		    switch(zigzag_mode) //判断当前模式
		    {

		    case Zigzag_Manual:
		    	//打印位置测试用

		#if 1
		    	//gcs().send_text(MAV_SEVERITY_WARNING,"ZIG_Manual RUN1");
		        zigzag_manual_control();
		#else
		    	//gcs().send_text(MAV_SEVERITY_WARNING,"ZIG_Manual RUN2");
		        poshold_run();
		#endif
		    	break;

		    case Zigzag_Auto:

		    	float closest_angle,closest_distance;
		    	 float avoid_direction_temp;
		    	//float  distante_object;
		    	 //临时存储数据
		    	float avoid_direction,avoid_distance;
		    	time_Lidar++;
		    	//20Hz处理一下雷达数据
		    	if(time_Lidar==20)
		    	{
		    		 copter.g2.proximity.get_closest_object(closest_angle,closest_distance);

		    		 //获取障碍物距离，躲避障碍物的方向和距离
		    		 //fly_direction 在后面计算，是否存在问题，风险，当心！！！
		    		 if(fly_direction==1)
		    		 copter.g2.proximity.get_object_front(Lidar_distante_object,avoid_direction,avoid_distance);

		    		 if(fly_direction==-1)
		    		 copter.g2.proximity.get_object_back(Lidar_distante_object,avoid_direction,avoid_distance);

		    		 //距离障碍物太近，发生危险，退出自动模式
		    		if(closest_distance<1.3&&closest_distance>0.7)
		    		{
		    			//先关掉，风险，当心！！！
		    		//zigzag_mode = Zigzag_Manual;
		    		AP_Notify::flags.zigzag_record_mode=0;
		    		gcs().send_text(MAV_SEVERITY_WARNING,"Zigzag manual closest_distance"); //发送自动信息
		    		zigzag_auto_complete_state = false;
		    		 #if 1
		    		 loiter_nav->init_target();
		    		 #else
		    		 poshold_init(true);
		    		#endif
		    		zigzag_auto_stop();
		    		return;
		    		}


		    		time_Lidar=0;

		    		//风险，小心，测试用  障碍物标志关掉了
		    		//如果障碍物直接在3m内，有问题，风险，当心！！！
		    		if(Lidar_distante_object<5&&Lidar_distante_object>3)
		    		//if(Lidar_distante_object<5&&Lidar_distante_object>3&&!zigzag_waypoint_state.meet_obstacle)
		    		{

		    			//后面要关闭这里，风险，小心
		    			zigzag_waypoint_state.meet_obstacle=0;


		    			avoid_distance_sum+=avoid_distance;
		    			avoid_directon_sum+=avoid_direction;
		    			time_data_handle++;

		    			//防止数据溢出
		    			if(time_data_handle>1000)
		    			{
		    				avoid_distance_sum=0;
		    				avoid_directon_sum=0;
		    				time_data_handle=0;
		    			}
		    		}

		    		//障碍物小于5m,计算避障距离和方向，障碍物标志位置位 只能计算一次
		    		//if(Lidar_distante_object<3)//风险，小心，测试用  障碍物标志关掉了
		    		if(Lidar_distante_object<3&&!zigzag_waypoint_state.meet_obstacle)
		    		{
		    			//后面要打开这里，风险，小心
		    			zigzag_waypoint_state.meet_obstacle=1;

		    			//只能计算一次，否则数值失控  Lidar_avoid_distance
		    			Lidar_avoid_distance=avoid_distance_sum/time_data_handle;
		    			avoid_direction_temp=avoid_directon_sum/time_data_handle;
		    			if(avoid_direction_temp>0)
		    				Lidar_avoid_direction=1;
		    			else
		    				Lidar_avoid_direction=-1;

		    			//数据需要重新复位
		    			avoid_distance_sum=0;
		    			avoid_directon_sum=0;
		    			time_data_handle=0;

		    		}


		    	}


		    	//打印位置测试用
		    		next_temp++;
		    		if(next_temp>1000)
		    		{
		    			next_temp=0;
		    			gcs().send_text(MAV_SEVERITY_INFO, "distante_object=%f", (double)Lidar_distante_object);
		    			gcs().send_text(MAV_SEVERITY_INFO, "avoid_direction: =%f,avoid_distance=%f", (double)Lidar_avoid_direction,(double)Lidar_avoid_distance);
		    			//gcs().send_text(MAV_SEVERITY_INFO, "zigzag_waypoint_state=%f", (double)zigzag_waypoint_state.flag);
		    			//gcs().send_text(MAV_SEVERITY_INFO, "fly_direction=%f",  (double)fly_direction);

		    			if(zigzag_waypoint_state.meet_obstacle)
		    				{
		    				//先关掉
		    				gcs().send_text(MAV_SEVERITY_WARNING, "There is a obstacle in the front");
		    			    AP_Notify::flags.zigzag_record = 16;
		    				}

		    				else
		    				{
		    					gcs().send_text(MAV_SEVERITY_WARNING, "We are away from obstacle  ");
		    				    AP_Notify::flags.zigzag_record = 16;
		    					}
		    		}



		    	/*
		    	  //获取左前方和右前方障碍物的角度和距离
		    	int8_t i;
		    	float object__angle_temp,object_distance_temp;
		    	float closest_angle,closest_distance;
               //读取激光雷达的数据
		    	for (i=0;i<8;i++)
		    	{
		    	copter.g2.proximity.get_object_angle_and_distance(i,object__angle_temp,object_distance_temp);
		    	if(object_distance_temp<=0.5)
		    	{
		    		object_distance_temp=133;//获得的数据很小，说明障碍物很远
		    	}

		    	copter.g2.proximity.get_closest_object(closest_angle,closest_distance);
		    	//距离障碍物小于7米，锁住飞行方向
		    	if(closest_distance<7&&closest_distance>0.7)
		    		fly_dir_lock=1;
		    	else
		    		fly_dir_lock=0;

		    	//距离障碍物太近，发生危险，退出自动模式
		    	if(closest_distance<1.3&&closest_distance>0.7)
		    	{

		        	zigzag_mode = Zigzag_Manual;
		        	AP_Notify::flags.zigzag_record_mode=0;
		        	gcs().send_text(MAV_SEVERITY_WARNING,"Zigzag manual closest_distance"); //发送自动信息



		        	zigzag_auto_complete_state = false;
		    #if 1
		            loiter_nav->init_target();
		    #else
		            poshold_init(true);
		    #endif
		            zigzag_auto_stop();
		        	return;

		    	}




		    	object_angle[i]=object__angle_temp;
		    	object_distance[i]=object_distance_temp;

		    	//测试用，打印各个扇区的障碍物信息
		    	if(next_temp==0)
		    	gcs().send_text(MAV_SEVERITY_INFO, "object: sector=%f  distance=%f", (double)i, (double)object_distance_temp);
		    	}
		    	//飞行方向在后面计算，是否有问题，风险，要当心！！！
		    	//fly_direction=1;//测试用 风险，要当心！
		    	//激光雷达重新安装，位置改变了， 需要改动扇区，风险，要当心！，雷达向右边偏转45度
		        switch(fly_direction)
		        {
		        case 1:

				if(object_distance[6]<5||object_distance[7]<5||object_distance[0]<5)
				    {	//飞行方向顺着机头，利用7，0,1扇区来避障
		        if(object_distance[6]<object_distance[7]&&object_distance[6]<object_distance[0])
		        	avoid_direction=1;
		        else
		        	avoid_direction=-1;

		        zigzag_waypoint_state.meet_obstacle=1;
		        object_detect_number=0;
		            }

				else
				{
					object_detect_number++;
					//没有障碍物探测次数大于临界值，才确定没有碰到障碍物，因为雷达存在漏检情况
					if(object_detect_number>7)
					{
						object_detect_number=0;
	                zigzag_waypoint_state.meet_obstacle=0;
					}
				}
		        break;

		        case -1:
		        	//存在问题，如果飞机垂直AB航线，也会发现障碍物！！！  风险，要当心！！！

		        	//飞行方向顺着机尾， 利用3, 4，5 扇区来避障
		        	if(object_distance[2]<5||object_distance[3]<5||object_distance[4]<5)
		        	   {
		        			        if(object_distance[2]<object_distance[3]&&object_distance[2]<object_distance[4])
		        			        	avoid_direction=1;
		        			        else
		        			        	avoid_direction=-1;
		        			        zigzag_waypoint_state.meet_obstacle=1;
		        			        object_detect_number=0;
		        	    }

		        	else
		        				{
		        					object_detect_number++;
		        					//没有障碍物探测次数大于临界值，才确定没有碰到障碍物，因为雷达存在漏检情况
		        					if(object_detect_number>7)
		        					{
		        					object_detect_number=0;
		        	                zigzag_waypoint_state.meet_obstacle=0;
		        					}
		        				}
		         break;
		        default:break;
		        }
		    	//zigzag_bearing//根据当前的航向角来决定飞机的前方


*/


		    	//躲避障碍物
/*
		    	    uint16_t rc9_in = RC_Channels::rc_channel(CH_9)->get_radio_in();
		    		if(rc9_in>1500)
		    		{

		    		zigzag_waypoint_state.meet_obstacle=1;
		    		}
		    		else
		    			zigzag_waypoint_state.meet_obstacle=0;

*/

		        //测试，有重大变动，当心，风险
		    	//if(zigzag_auto_complete_state||(zigzag_waypoint_state.meet_obstacle&&!zigzag_waypoint_state.obstacle_flag)) //第一次：zigzag_auto_complete_state=1，zigzag_change_yaw=0;


		    	if(zigzag_auto_complete_state && !zigzag_change_yaw||(zigzag_waypoint_state.meet_obstacle&&!zigzag_waypoint_state.obstacle_flag)) //第一次：zigzag_auto_complete_state=1，zigzag_change_yaw=0;
		    	{

		    		state_zigzag=1;//测试用
		    		gcs().send_text(MAV_SEVERITY_INFO, "state_zigzag=%f", (double)state_zigzag);


		    		AP_Notify::flags.avoid_course++;
		    		if(AP_Notify::flags.avoid_course>4)
		    			AP_Notify::flags.avoid_course=0;

		    		//没有遇见障碍物，航线不变
		    	    if(!zigzag_waypoint_state.meet_obstacle)
		    		{
		    		 //zigzag_waypoint_state.index++;
		    		zigzag_waypoint_state.obstacle_flag=0;
		    			}

		    		 else
		    	{
		    	 zigzag_waypoint_state.obstacle_flag++;
		    	 //遇见障碍物以后，走完三个避障点，继续往前走，
		    	 //注意index的衔接，有可能走到之前的航点，风险，当心！！！
		    	 if(zigzag_waypoint_state.obstacle_flag==3)
		    	 {
		    		 //复位应该放在计算避障点之后
		    	// zigzag_waypoint_state.obstacle_flag=0;
		         zigzag_waypoint_state.index=zigzag_waypoint_state.index-1;//保护最后的航点
		         //躲开障碍物之前，标志位不做处理

		         //测试，有重大变动，当心，风险 后面要打开
		         //zigzag_waypoint_state.meet_obstacle=0;
		    		}
		    	}



		           wp_nav->wp_and_spline_init();
		           zigzag_set_destination();
		           zigzag_auto_complete_state = false;

		    	}
		    	else
		    	{
		    		zigzag_auto_control();
		    	}
		    	break;
		    }
}

/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_manual_control()
*函数功能：手动控制模式
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息：zigzag_manual_control - process manual control
*************************************************************************************************************************/
void Copter::ModeZigZag::zigzag_manual_control()
{
	float target_yaw_rate = 0.0f;
	    float target_climb_rate = 0.0f;
	    float target_roll = 0.0f;
	    float target_pitch = 0.0f;

	    // initialize vertical speed and acceleration's range
	    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
	    pos_control->set_accel_z(g.pilot_accel_z);
	    // process pilot inputs unless we are in radio failsafe
	    if (!copter.failsafe.radio)
	    {
	        // apply SIMPLE mode transform to pilot inputs
	        update_simple_mode();
	        // convert pilot input to lean angles
	        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
	        // process pilot's roll and pitch input
	        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
	        // get pilot's desired yaw rate
	        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

	        // get pilot desired climb rate
	        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
	        //make sure the climb rate is in the given range, prevent floating point errors
	        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
	    }
	    else
	    {
	        // clear out pilot desired acceleration in case radio failsafe event occurs and we
	        //do not switch to RTL for some reason
	    	loiter_nav->clear_pilot_desired_acceleration();
	    }

	    // set motors to full range
	    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
	    // run loiter controller
	    loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

	    // call attitude controller
	    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(),loiter_nav->get_pitch(), target_yaw_rate/*, get_smoothing_gain()*/);

	    // adjust climb rate using rangefinder
	    if (copter.rangefinder_alt_ok())
	    {
	        // if rangefinder is ok, use surface tracking
	       target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
	    }

	    // get avoidance adjusted climb rate
	    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

	    // update altitude target and call position controller
	    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
	    //adjusts target up or down using a climb rate

	    pos_control->update_z_controller();
}


/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_auto_control()
*函数功能：自动控制模式
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息：zigzag_auto auto run
*************************************************************************************************************************/

void Copter::ModeZigZag::zigzag_auto_control()
{
    // process pilot's yaw input
	float target_roll = 0, target_pitch = 0;
    float target_yaw_rate = 0;
    float target_climb_rate = 0.0f;




    if (!copter.failsafe.radio)
    {
        // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    	get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
        //get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate))
        {
        	auto_yaw.set_mode(AUTO_YAW_HOLD); //设定自动偏航保持

        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }


     // zigzag mode direction
     if(zigzag_waypoint_state.direct == 0)
    {
    	switch(zigzag_rc_state)
    	{
    	case RC_MID:
    		if(target_roll > copter.aparm.angle_max /10)
    		{

    			zigzag_rc_state = RC_RIGHT;
    		}

    		else if(target_roll < -copter.aparm.angle_max /10)
    		{

    			zigzag_rc_state = RC_LEFT;
    		}

    		break;
    	case RC_RIGHT:
    		if((target_roll<=(copter.aparm.angle_max / 100))&&(target_roll>=(-copter.aparm.angle_max / 100)))
    		{
    			zigzag_waypoint_state.direct = -1;
    		}

    		break;
    	case RC_LEFT:
    		if((target_roll<=(copter.aparm.angle_max / 100))&&(target_roll>=(-copter.aparm.angle_max / 100)))
    		{

    			zigzag_waypoint_state.direct = 1;

    		}
    		break;
    	}

    	if(zigzag_waypoint_state.direct != 0)
    	{
    		zigzag_auto_complete_state = true;  //到这里才是真正的执行
    		zigzag_waypoint_state.flag = 0x05;

    		return;
    	}
    }


     /*|| !is_zero(target_yaw_rate)*/
   // 测试用 先屏蔽，重大改动，风险，当心
   else if(!is_zero(target_roll) || !is_zero(target_pitch) )
    {
    	zigzag_mode = Zigzag_Manual;

    	//测试用
    	AP_Notify::flags.zigzag_record_mode=0;
    	gcs().send_text(MAV_SEVERITY_WARNING,"Zigzag manual target_roll"); //发送自动信息


    	zigzag_auto_complete_state = false;
#if 1
        loiter_nav->init_target();
#else
        poshold_init(true);
#endif
        zigzag_auto_stop();
    	return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller to update xy
    copter.failsafe_terrain_set_status(wp_nav->update_zigzag_wpnav());

    // adjust climb rate using rangefinder
    if (copter.rangefinder_alt_ok())
    {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
    }

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    //hal.console->printf("taget climb rate:%f\n", target_climb_rate);
    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();


     //call attitude controller


    if (auto_yaw.mode()  == AUTO_YAW_HOLD)
    {
         //roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), /*target_yaw_rate*/0/*, get_smoothing_gain()*/);
    }
    else if (auto_yaw.mode()  == AUTO_YAW_RATE)
    {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
    	 attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(),auto_yaw.rate_cds());

    }
    else
    {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
       attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true/*, get_smoothing_gain()*/);

    }

    //测试用
   //

    if(zigzag_waypoint_state.direct != 0 && ((zigzag_waypoint_state.flag & 0x05) == 0x05))
    {
    	zigzag_auto_complete_state = wp_nav->reached_wp_destination();        //没有到就返回0，到了就返回1

    	//state_zigzag=3;//测试用
    	//gcs().send_text(MAV_SEVERITY_INFO, "state_zigzag=%f", (double)state_zigzag);


    	if(zigzag_change_yaw && copter.mode_auto.verify_yaw())
		{
			zigzag_change_yaw = false;

		}
		if(zigzag_auto_complete_state && zigzag_waypoint_state.bp_mode != Zigzag_None)
		{
			zigzag_change_yaw = true;
			auto_yaw.set_mode(AUTO_YAW_FIXED); //设定自动偏航保持
			auto_yaw.set_fixed_yaw(zigzag_bearing*0.01f,0,0,0);
			zigzag_waypoint_state.bp_mode = Zigzag_None;
			zigzag_waypoint_state.index--;

			//这里提示断点模式，每次进入会闪烁两下红灯
			if(zigzag_change_yaw==1)
			{

				AP_Notify::flags.zigzag_record_mode_erro=4;
			}
			else
			{
				AP_Notify::flags.zigzag_record_mode_erro=0;

			}
		}

	}
}

/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_calculate_next_dest(Vector3f& next, uint16_t index)
*函数功能：设置下一目标点
*修改日期：2018-10-8
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::ModeZigZag::zigzag_set_destination(void)
{
	Vector3f next;  //改成全局变量，测试用  风险，要当心
	Vector3f cur_pos;
	Vector3f v_BP2Next;
	Vector3f v_BP2Next_uint;
	Vector3f v_BP2Cur;
	Vector3f v_BP2E;
	float track_length;
	float dotproduct;





	switch(zigzag_waypoint_state.bp_mode)
	{
	case Zigzag_None:
		//没有遇见障碍物，旧的航线索引不变
		if(!zigzag_waypoint_state.obstacle_flag)
	    zigzag_waypoint_state.index++;

		state_zigzag=2;//测试用
		gcs().send_text(MAV_SEVERITY_INFO, "state_zigzag=%f", (double)state_zigzag);


		zigzag_calculate_next_dest(next, zigzag_waypoint_state.index);


		wp_nav->set_wp_destination(next, false);



		// switch sprayer run on or off

		copter.sprayer.run(zigzag_waypoint_state.index%2 == 0);

		break;

	// no power or no drug will record breakpoint
	case Zigzag_PowerNone:
	case Zigzag_DrugNone:
	case Zigzag_ModeSwitch:

		wp_nav->set_wp_destination(zigzag_waypoint_state.vBP_pos, false);

		zigzag_waypoint_state.flag = 0x05;
		// stop sprayer
		copter.sprayer.run(false);
		break;


	case Zigzag_PilotOverride:
		//Vector3f next;
		zigzag_calculate_next_dest(next, zigzag_waypoint_state.index);
		next.z = zigzag_waypoint_state.vBP_pos.z;

		switch(zigzag_waypoint_state.index%4)
		{
		case 1:
		case 3:

		      break;

		case 2:
		case 0:
			cur_pos = inertial_nav.get_position();
			v_BP2Next = next - zigzag_waypoint_state.vBP_pos;
			v_BP2Next.z = 0;
			track_length = v_BP2Next.length();
			if(is_zero(track_length))
			{
				break;
			}
			v_BP2Next_uint = v_BP2Next / track_length;
			//v_BP2Next.z = 0;
			v_BP2Cur = cur_pos - zigzag_waypoint_state.vBP_pos;
			//v_BP2Cur.z = 0;
			dotproduct = v_BP2Next_uint.x * v_BP2Cur.x + v_BP2Next_uint.y * v_BP2Cur.y;
			if(dotproduct > 0)
			{

				if(dotproduct < track_length)
				{
					v_BP2E =  v_BP2Next_uint * dotproduct;
					next = v_BP2E + zigzag_waypoint_state.vBP_pos;
				}

			}
			else
			{
				next = zigzag_waypoint_state.vBP_pos;
			}

			break;
		}
		wp_nav->set_wp_destination(next,false);

		// stop sprayer
		copter.sprayer.run(false);

		break;
	}
}


/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_calculate_next_dest(Vector3f& next, uint16_t index)
*函数功能：设置下一目标点
*修改日期：2018-10-8
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::ModeZigZag::zigzag_calculate_next_dest(/*Location_Class& dest*/Vector3f& next, uint16_t index)
{
	Vector3f v_A2B  = zigzag_waypoint_state.vB_pos - zigzag_waypoint_state.vA_pos;
	//Vector3f last_next,bearing_error;
	v_A2B.z = 0;

	float dist_AB = v_A2B.length();
	float a1 = v_A2B.x;
	float b1 = v_A2B.y;
	float c1 = 0.0f;//v_A2B.x * zigzag_waypoint_state.vB_pos.x + v_A2B.y * zigzag_waypoint_state.vB_pos.y;
	float a2=0, b2=0, c2 = 0;

    //gcs().send_text(MAV_SEVERITY_INFO, "ModeZigZag: rc9_in=%f meet_obstacle=%f", (double)rc9_in, (double)zigzag_waypoint_state.meet_obstacle);

	switch(index%4)
	{
	case 1:
	case 0:

		c1 = v_A2B.x * zigzag_waypoint_state.vB_pos.x + v_A2B.y * zigzag_waypoint_state.vB_pos.y;
		c2 = zigzag_waypoint_state.direct * dist_AB * zigzag_waypoint_state.width * ((index+1)>>1);
		c2 = c2 + zigzag_waypoint_state.vB_pos.x * v_A2B.y - zigzag_waypoint_state.vB_pos.y * v_A2B.x;
		a2 = v_A2B.y;
		b2 = -v_A2B.x;
		break;

	case 2:
	case 3:

		c1 = v_A2B.x * zigzag_waypoint_state.vA_pos.x + v_A2B.y * zigzag_waypoint_state.vA_pos.y;
		c2 = zigzag_waypoint_state.direct * dist_AB * zigzag_waypoint_state.width * ((index+1)>>1);
		c2 = c2 + zigzag_waypoint_state.vA_pos.x * v_A2B.y - zigzag_waypoint_state.vA_pos.y * v_A2B.x;
		a2 = v_A2B.y;
		b2 = -v_A2B.x;
		break;
	}




	//Vector3f next;
	float denominator = (a1 * b2 - a2 * b1);

	if(!is_zero(denominator))
	{
		next.x = (c1 * b2 - c2 * b1) / denominator;
		next.y = -(c1 * a2 - c2 * a1) / denominator;
	}



/*
	//飞行方向没有锁定才计算
	  if(!fly_dir_lock)
	  {
		  //简单替代
		next_location=next;
		next_location.z=0;

		last_next=next_location-last_location;
	  }

		bearing_error=last_next-v_A2B;

*/


		//float dist_direction=bearing_error.length();
		//(last_next-v_A2B).length();
		//飞行方向和AB方向相同、
		//没有遇到障碍物时才判断飞行方向，如果飞机离开障碍物，机尾再次发现障碍物
		//导致再一次更改航线，飞行方向的判断过于单一，风险，当心！！！
		//完成一次避障飞行前，方向必须锁定

		/*
		 if(!zigzag_waypoint_state.obstacle_flag)
		{
		if(dist_direction<9)
			fly_direction=1;
			//飞行方向和AB方向不相同
			else
				fly_direction=-1;

				  gcs().send_text(MAV_SEVERITY_INFO, " last_location.x=%f next_location.x=%f", (double)last_location.x, (double)next_location.x);
			      gcs().send_text(MAV_SEVERITY_INFO, "dist_direction=%f fly_direction=%f", (double)dist_direction, (double)fly_direction);

		}


		//飞行方向没有锁定才计算
		 if(!fly_dir_lock)
		 {
	//记录飞行过的位置
	  last_location.x=next.x;
	  last_location.y=next.y;
	  last_location.z=0;
		 }
*/
		 //根据index直接判断飞行方向

if(index%4==2)
	fly_direction=-1;
if(index%4==0)
	fly_direction=1;


	if(index==1)
	{
	zigzag_waypoint_state.vC_pos.x=next.x;
	zigzag_waypoint_state.vC_pos.y=next.y;
	}

	//遇到障碍物，重新规划航线
	//一直在计算，可能有问题，风险，当心！！！
	Vector3f obstacle_break=inertial_nav.get_position();


	//Vector3f virtual_obstacle,avoid_point;
	float right_away=500,front_away=1000;//单位是cm
	right_away=Lidar_avoid_distance;
	front_away= right_away*2;

	//逆时针旋转当前点到虚拟y轴 A点成为原点（0，0）
	obstacle_break.z = 0;
	//float dist_obstacle_break = obstacle_break.length();
	//AB线与X轴的夹角为α，障碍物点与X轴的夹角为β
	//cos（α+β）=cosαcosβ-sinαsinβ
	//sin （α+β）=sinαcosβ+cosαsinβ

	//virtual_obstacle.x=dist_obstacle_break*((v_A2B.y/dist_AB)*(obstacle_break.x/dist_obstacle_break)-(v_A2B.x/dist_AB)*(obstacle_break.y/dist_obstacle_break));
	//virtual_obstacle.y=dist_obstacle_break*((v_A2B.x/dist_AB)*(obstacle_break.x/dist_obstacle_break)+(v_A2B.y/dist_AB)*(obstacle_break.y/dist_obstacle_break));


//加入躲避障碍物方向 （avoid_direction）后，有bug,风险，要当心！！！
//avoid_direction 负责左右运动
//fly_direction 负责前后运动
//目前只是往右边打点的情况，往左边打点不一样，风险，要当心！！！
	switch(zigzag_waypoint_state.obstacle_flag)
		{
		case 1:
			  zigzag_waypoint_state.avoidA_pos.x=obstacle_break.x+Lidar_avoid_direction*right_away/zigzag_waypoint_state.width*(zigzag_waypoint_state.vC_pos.x-zigzag_waypoint_state.vB_pos.x);
			  zigzag_waypoint_state.avoidA_pos.y=obstacle_break.y+Lidar_avoid_direction*right_away/zigzag_waypoint_state.width*(zigzag_waypoint_state.vC_pos.y-zigzag_waypoint_state.vB_pos.y);
			  next.x = zigzag_waypoint_state.avoidA_pos.x;
			  next.y = zigzag_waypoint_state.avoidA_pos.y;

			  gcs().send_text(MAV_SEVERITY_INFO, " vA_pos.x=%f vA_pos.y=%f", (double)zigzag_waypoint_state.vA_pos.x, (double)zigzag_waypoint_state.vA_pos.y);
	          gcs().send_text(MAV_SEVERITY_INFO, "vB_pos.x=%f vB_pos.y=%f", (double)zigzag_waypoint_state.vB_pos.x, (double)zigzag_waypoint_state.vB_pos.y);
	          gcs().send_text(MAV_SEVERITY_INFO, "vC_pos.x=%f vC_pos.y=%f", (double)zigzag_waypoint_state.vC_pos.x, (double)zigzag_waypoint_state.vC_pos.y);

	          gcs().send_text(MAV_SEVERITY_INFO, "obstacle_break.x=%f obstacle_break.y=%f", (double)obstacle_break.x, (double)obstacle_break.y);
	          gcs().send_text(MAV_SEVERITY_INFO, "avoidA_pos.x=%f avoidA_pos.y=%f", (double)zigzag_waypoint_state.avoidA_pos.x, (double)zigzag_waypoint_state.avoidA_pos.y);

			//avoid_point.x=virtual_obstacle.x+right_away;
		      //avoid_point.y=virtual_obstacle.y;

		      break;
		case 2:
			   zigzag_waypoint_state.avoidB_pos.x=zigzag_waypoint_state.avoidA_pos.x+fly_direction*front_away/dist_AB*v_A2B.x;
			   zigzag_waypoint_state.avoidB_pos.y=zigzag_waypoint_state.avoidA_pos.y+fly_direction*front_away/dist_AB*v_A2B.y;
			   next.x = zigzag_waypoint_state.avoidB_pos.x;
			   next.y = zigzag_waypoint_state.avoidB_pos.y;

			  //avoid_point.x=virtual_obstacle.x+right_away;
			  //avoid_point.y=virtual_obstacle.y+front_away;
			  break;
		case 3:
			//两个不一样的BC点，容易混淆，当心
			   zigzag_waypoint_state.avoidC_pos.x=zigzag_waypoint_state.avoidB_pos.x-Lidar_avoid_direction*right_away/zigzag_waypoint_state.width*(zigzag_waypoint_state.vC_pos.x-zigzag_waypoint_state.vB_pos.x);
			   zigzag_waypoint_state.avoidC_pos.y=zigzag_waypoint_state.avoidB_pos.y-Lidar_avoid_direction*right_away/zigzag_waypoint_state.width*(zigzag_waypoint_state.vC_pos.y-zigzag_waypoint_state.vB_pos.y);
			   next.x = zigzag_waypoint_state.avoidC_pos.x;
			   next.y = zigzag_waypoint_state.avoidC_pos.y;

			   //复位标志量
			   zigzag_waypoint_state.obstacle_flag=0;
			 //avoid_point.x=virtual_obstacle.x;
			 //avoid_point.y=virtual_obstacle.y+front_away;
			 break;
		default:break;
		}

	//顺时针旋转，回到真实点
    //cos（α-β）=cosαcosβ+sinαsinβ
	//sin（α-β）=sinαcosβ-cosαsinβ
	/*
	avoid_point.z=0;
	float dist_avoid_point = avoid_point.length();

	if(zigzag_waypoint_state.obstacle_flag)
	{
		next.x =zigzag_waypoint_state.vA_pos.x+dist_avoid_point*((v_A2B.y/dist_AB)*(avoid_point.x/dist_avoid_point)+(v_A2B.y/dist_AB)*(avoid_point.x/dist_avoid_point));

		next.y = zigzag_waypoint_state.vA_pos.y+dist_avoid_point*((v_A2B.x/dist_AB)*(avoid_point.x/dist_avoid_point)-(v_A2B.y/dist_AB)*(avoid_point.y/dist_avoid_point));
	}
*/
	next_printf=next;//测试用
	gcs().send_text(MAV_SEVERITY_INFO, "ModeZigZag: index=%f obstacle_flag=%f", (double)zigzag_waypoint_state.index, (double)zigzag_waypoint_state.obstacle_flag);
	gcs().send_text(MAV_SEVERITY_INFO, "ModeZigZag: next.x=%f next.y=%f", (double)next.x, (double)next.y);





	next.z = inertial_nav.get_position().z;

}


/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_set_bp_mode(ZigzagBPMode bp_mode)
*函数功能：设置AB点模式
*修改日期：2018-10-8
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::ModeZigZag::zigzag_set_bp_mode(ZigzagBPMode bp_mode)
{
	zigzag_waypoint_state.bp_mode = bp_mode;
}

/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_stop()
*函数功能：自动阻止
*修改日期：2018-10-8
*修改作者：cihang_uav
*备注信息：
*************************************************************************************************************************/
void Copter::ModeZigZag::zigzag_stop()
{
	if(zigzag_mode == Zigzag_Auto && zigzag_rc_state != RC_MID)
	{
		zigzag_auto_stop();
		zigzag_waypoint_state.bp_mode = Zigzag_ModeSwitch;
		return;
	}
	else if((zigzag_waypoint_state.flag & 0x05) == 0x05)
	{

		zigzag_waypoint_state.bp_mode = Zigzag_PilotOverride;
	}

}

/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_auto_stop()
*函数功能：自动阻止
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息：zigzag auto stop, stop auto run and record current position as breakpoint position
*************************************************************************************************************************/

void Copter::ModeZigZag::zigzag_auto_stop()
{
	if(zigzag_waypoint_state.bp_mode == Zigzag_None)
	{
	  zigzag_waypoint_state.vBP_pos = inertial_nav.get_position();
	  zigzag_waypoint_state.bp_pos = copter.current_loc;
	}

}


/***********************************************************************************************************************
*函数原型：bool Copter::ZigZag::zigzag_record_point(bool aPoint)
*函数功能：记录AB点信息
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息：zigzag record A point or B point; aPoint==true record A point; aPoint == false record B point
*************************************************************************************************************************/

bool Copter::ModeZigZag::zigzag_record_point(bool aPoint)
{
//	 hal.uartG->printf("AAA\r\n");
	bool ret = false;
	const Vector3f& vel = inertial_nav.get_velocity();
	float vel_horizontal = norm(vel.x, vel.y);
	// position not healthy then return, no recording
	// before record point, horizontal velocity must less than 1m/s
	if(!copter.position_ok() || vel_horizontal > 100) //定位ok,并且运行速度很小，才可以
	{
//		 hal.uartG->printf("AAABBB\r\n");
		return false;
	}
	// record A point
	if(aPoint)
	{
		// clear all record
		zigzag_clear_record();

		zigzag_waypoint_state.vA_pos = inertial_nav.get_position();
		zigzag_waypoint_state.a_pos = copter.current_loc;

		// After record A point, clear B point flag
		zigzag_waypoint_state.a_hasbeen_defined = true;

		ret = true;
	}
	// before record B point, A point must be recorded
	else if(zigzag_waypoint_state.a_hasbeen_defined)
	{
		zigzag_waypoint_state.vB_pos = inertial_nav.get_position();
		zigzag_waypoint_state.b_pos = copter.current_loc;
		zigzag_waypoint_state.b_hasbeen_defined = true;
		ret = true;
	}
	return ret;
}






/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_clear_record(void)
*函数功能：清除所有的参数
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息：clear all record
*************************************************************************************************************************/
void Copter::ModeZigZag::zigzag_clear_record(void)
{
	// set all parameter 0
	g.Zigzag_time.set_and_save(0);

    g2.ab_index.set_and_save(0);
    g2.ab_dirct.set_and_save(0);
    g2.aPos_lat.set_and_save(0);
    g2.aPos_lng.set_and_save(0);
    g2.aPos_alt.set_and_save(0);
    g2.bPos_lat.set_and_save(0);
    g2.bPos_lng.set_and_save(0);
    g2.bPos_alt.set_and_save(0);
    g2.bpPos_lat.set_and_save(0);
    g2.bpPos_lng.set_and_save(0);
    g2.bpPos_alt.set_and_save(0);
    g2.ab_bpMode.set_and_save(0);

    zigzag_waypoint_state.a_hasbeen_defined = false;
    zigzag_waypoint_state.b_hasbeen_defined = false;
    zigzag_waypoint_state.direct = 0;
    zigzag_waypoint_state.index = 0;
    zigzag_waypoint_state.bp_mode = Zigzag_None;
    zigzag_waypoint_state.width = g.Zigzag_width * 100; // convert to cm
}

/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_save(void)
*函数功能：保存数据
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息：save record
*************************************************************************************************************************/
void Copter::ModeZigZag::zigzag_save(void)
{
	// record time
    uint64_t gps_timestamp = copter.gps.time_epoch_usec();
    int32_t cur_timestamp_min = gps_timestamp / 6.0e7f;
    g.Zigzag_time.set_and_save(cur_timestamp_min);

    g2.ab_index.set_and_save(zigzag_waypoint_state.index);
    g2.ab_dirct.set_and_save(zigzag_waypoint_state.direct);
    g2.aPos_lat.set_and_save(zigzag_waypoint_state.a_pos.lat);
    g2.aPos_lng.set_and_save(zigzag_waypoint_state.a_pos.lng);
    g2.aPos_alt.set_and_save(zigzag_waypoint_state.a_pos.alt);
    g2.bPos_lat.set_and_save(zigzag_waypoint_state.b_pos.lat);
    g2.bPos_lng.set_and_save(zigzag_waypoint_state.b_pos.lng);
    g2.bPos_alt.set_and_save(zigzag_waypoint_state.b_pos.alt);
    g2.bpPos_lat.set_and_save(zigzag_waypoint_state.bp_pos.lat);
    g2.bpPos_lng.set_and_save(zigzag_waypoint_state.bp_pos.lng);
    g2.bpPos_alt.set_and_save(zigzag_waypoint_state.bp_pos.alt);

	switch(zigzag_waypoint_state.bp_mode)
	{
	case Zigzag_None:
		g2.ab_bpMode.set_and_save(0);
		break;
	case Zigzag_PowerNone:
		g2.ab_bpMode.set_and_save(1);
		break;
	case Zigzag_DrugNone:
		g2.ab_bpMode.set_and_save(2);
		break;
	case Zigzag_ModeSwitch:
		g2.ab_bpMode.set_and_save(3);
		break;
	case Zigzag_PilotOverride:
		g2.ab_bpMode.set_and_save(4);
		break;
	}

}

/***********************************************************************************************************************
*函数原型：void Copter::ZigZag::zigzag_load(void)
*函数功能：加载参数
*修改日期：2018-9-26
*修改作者：cihang_uav
*备注信息：zigzag_auto auto run
*************************************************************************************************************************/
void Copter::ModeZigZag::zigzag_load(void)
{

	zigzag_waypoint_state.width = g.Zigzag_width * 100; // convert to cm
	zigzag_waypoint_state.index = g2.ab_index;
	zigzag_waypoint_state.direct = g2.ab_dirct;
	zigzag_waypoint_state.a_pos.lat = g2.aPos_lat;
	zigzag_waypoint_state.a_pos.lng = g2.aPos_lng;
	zigzag_waypoint_state.a_pos.alt = g2.aPos_alt;
	zigzag_waypoint_state.b_pos.lat = g2.bPos_lat;
	zigzag_waypoint_state.b_pos.lng = g2.bPos_lng;
	zigzag_waypoint_state.b_pos.alt = g2.bPos_alt;
	zigzag_waypoint_state.bp_pos.lat = g2.bpPos_lat;
	zigzag_waypoint_state.bp_pos.lng = g2.bpPos_lng;
	zigzag_waypoint_state.bp_pos.alt = g2.bpPos_alt;

	switch(g2.ab_bpMode)
	{
	case 0:
		zigzag_waypoint_state.bp_mode = Zigzag_None;
		break;
	case 1:
		zigzag_waypoint_state.bp_mode = Zigzag_PowerNone;
		break;
	case 2:
		zigzag_waypoint_state.bp_mode = Zigzag_DrugNone;
		break;
	case 3:
		zigzag_waypoint_state.bp_mode = Zigzag_ModeSwitch;
		break;
	case 4:
		zigzag_waypoint_state.bp_mode = Zigzag_ModeSwitch;//Zigzag_PilotOverride;
		break;
	}

	zigzag_waypoint_state.a_pos.get_vector_from_origin_NEU(zigzag_waypoint_state.vA_pos);
	zigzag_waypoint_state.b_pos.get_vector_from_origin_NEU(zigzag_waypoint_state.vB_pos);
	zigzag_waypoint_state.bp_pos.get_vector_from_origin_NEU(zigzag_waypoint_state.vBP_pos);
	zigzag_waypoint_state.vBP_pos.z = zigzag_waypoint_state.bp_pos.alt;

}


/************************************************************************************************************************************************
*                             File _end
*************************************************************************************************************************************************/


