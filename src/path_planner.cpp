#include "path_planner.h"
#include <iostream>

#define DEFAULT_SPEED_MPH 45
#define DELTA_SPEED_START_UP_MPH 3
#define DELTA_SPEED_MOVING_MPH 1
#define SENS_FUSION_CAR_ID 0
#define SENS_FUSION_CAR_VEL_DX 3
#define SENS_FUSION_CAR_VEL_DY 4
#define SENS_FUSION_CAR_S_POS 5
#define SENS_FUSION_CAR_D_POS 6
#define LEFT_MOST_LANE 0
#define MIDDLE_LANE 1
#define RIGHT_MOST_LANE 2
#define LANE_WIDTH 4
#define MIN_DIST_TO_SLOW_DOWN 35
#define MIN_DIST_TO_SLOW_DOWN_HEAVILY 20
#define MIN_DIST_TO_OVERTAKE_FRONT 40
#define MIN_DIST_TO_OVERTAKE_REAR_HIGH_SPEED 20
#define MIN_DIST_TO_OVERTAKE_REAR_MID_SPEED 35
#define MIN_DIST_TO_OVERTAKE_REAR_LOW_SPEED 50
#define LANE_KEEP_SMOOTH_FACTOR 1
#define LANE_CHANGE_SMOOTH_FACTOR 2
#define FALSE 0x00
#define TRUE 0x01


Path_Planner::Path_Planner(const vector<double> &maps_s_in, const vector<double> &maps_x_in, const vector<double> &maps_y_in,short path_size_in)
{
	for(int i=0; i<maps_s_in.size();i++)
	{
		maps_s.push_back(maps_s_in[i]);
		maps_x.push_back(maps_x_in[i]);
		maps_y.push_back(maps_y_in[i]);
	}
	
	current_state = STATE_START_UP;
	path_size = path_size_in;
	p_path_generator = new Path_Gen(maps_s,maps_x,maps_y);
	id_of_vehicle_ahead = 255;
	overtake_target_lane = 255;
}

vector<point> Path_Planner::PathUpdate(struct vehicle_current_state vehicle_state,
							const vector<double> &previous_path_x,
							const vector<double> &previous_path_y,
							const vector<vector<double>> &sensor_fusion)
{
	vector<point> path;
	double sens_fusion_speed;
	double ego_vehicle_plan_speed;
	short ego_veh_lane;
	
	current_state = UpdateStateMAchine(vehicle_state,sensor_fusion);
	
	switch(current_state)
	{
		case STATE_START_UP:
			path = p_path_generator->GeneratePath(vehicle_state,
												  GetLaneNum(vehicle_state.d),
												  vehicle_state.speed+DELTA_SPEED_START_UP_MPH,
												  LANE_KEEP_SMOOTH_FACTOR,
												  previous_path_x,
												  previous_path_y);
			break;
		case STATE_KEEP_LANE:
			if(vehicle_state.speed < DEFAULT_SPEED_MPH)
			{
				ego_vehicle_plan_speed = vehicle_state.speed+(2*DELTA_SPEED_MOVING_MPH);
			}
			else
			{
				ego_vehicle_plan_speed = DEFAULT_SPEED_MPH;
			}
			
			path = p_path_generator->GeneratePath(vehicle_state,
												  GetLaneNum(vehicle_state.d),
												  ego_vehicle_plan_speed,
												  LANE_KEEP_SMOOTH_FACTOR,
												  previous_path_x,
												  previous_path_y);
			break;
		case STATE_PREPARE_CHANGE_LANE:
			sens_fusion_speed = distance(0,
										 0,
										 sensor_fusion[id_of_vehicle_ahead][SENS_FUSION_CAR_VEL_DX],
										 sensor_fusion[id_of_vehicle_ahead][SENS_FUSION_CAR_VEL_DY]);
			
			if((sensor_fusion[id_of_vehicle_ahead][SENS_FUSION_CAR_S_POS] - vehicle_state.s < MIN_DIST_TO_SLOW_DOWN_HEAVILY) &&
			   (vehicle_state.speed > sens_fusion_speed))
			{
				ego_vehicle_plan_speed = vehicle_state.speed-(2*DELTA_SPEED_MOVING_MPH);
			}
			else if((sensor_fusion[id_of_vehicle_ahead][SENS_FUSION_CAR_S_POS] - vehicle_state.s < MIN_DIST_TO_SLOW_DOWN) &&
			   (vehicle_state.speed > sens_fusion_speed))
			{
				ego_vehicle_plan_speed = vehicle_state.speed-DELTA_SPEED_MOVING_MPH;
			}
			else if((sensor_fusion[id_of_vehicle_ahead][SENS_FUSION_CAR_S_POS] - vehicle_state.s > MIN_DIST_TO_SLOW_DOWN_HEAVILY) &&
					(vehicle_state.speed < DEFAULT_SPEED_MPH))
			{
				ego_vehicle_plan_speed = vehicle_state.speed+DELTA_SPEED_MOVING_MPH;
			}
			else
			{
				ego_vehicle_plan_speed = DEFAULT_SPEED_MPH;
			}
			
			path = p_path_generator->GeneratePath(vehicle_state,
												  GetLaneNum(vehicle_state.d),
												  ego_vehicle_plan_speed,
												  LANE_KEEP_SMOOTH_FACTOR,
												  previous_path_x,
												  previous_path_y);
			
			break;
		case STATE_CHANGE_LANE_LEFT:
		case STATE_CHANGE_LANE_RIGHT:
			if(vehicle_state.speed < DEFAULT_SPEED_MPH)
			{
				ego_vehicle_plan_speed = vehicle_state.speed+(2*DELTA_SPEED_MOVING_MPH);
			}
			else
			{
				ego_vehicle_plan_speed = DEFAULT_SPEED_MPH;
			}
			
			path = p_path_generator->GeneratePath(vehicle_state,
												  overtake_target_lane,
												  ego_vehicle_plan_speed,
												  LANE_CHANGE_SMOOTH_FACTOR,
												  previous_path_x,
												  previous_path_y);
			break;
		default:
			break;
	}
	
	return path;
}

enum Path_Planner::path_planning_state Path_Planner::UpdateStateMAchine(
														struct vehicle_current_state vehicle_state,
														const vector<vector<double>> &sensor_fusion)
{
	enum path_planning_state new_state = current_state;
	short ego_veh_lane;
	short target_lane = 255;
	
	switch(current_state)
	{
		case STATE_START_UP:
			if(vehicle_state.speed >= DEFAULT_SPEED_MPH)
			{
				new_state = STATE_KEEP_LANE;
			}
			break;
		case STATE_KEEP_LANE:
			ego_veh_lane = GetLaneNum(vehicle_state.d);
			for(short i=0;i<sensor_fusion.size();i++)
			{
				if((ego_veh_lane == GetLaneNum(sensor_fusion[i][SENS_FUSION_CAR_D_POS])) &&
				   (sensor_fusion[i][SENS_FUSION_CAR_S_POS] > vehicle_state.s) &&
				   (sensor_fusion[i][SENS_FUSION_CAR_S_POS] - vehicle_state.s < MIN_DIST_TO_OVERTAKE_FRONT))
				{
					double sens_fusion_speed = distance(0,0,sensor_fusion[i][SENS_FUSION_CAR_VEL_DX],sensor_fusion[i][SENS_FUSION_CAR_VEL_DY]);
					
					// if the ego vehicle is faster than the other vehicle or the delta velocity < 3mph
					if(vehicle_state.speed - sens_fusion_speed >= -3)
					{
						new_state = STATE_PREPARE_CHANGE_LANE;
						id_of_vehicle_ahead = sensor_fusion[i][SENS_FUSION_CAR_ID];
						break;
					}
				}
			}
			break;
		case STATE_PREPARE_CHANGE_LANE:
			ego_veh_lane = GetLaneNum(vehicle_state.d);
			
			// check left overtake
			target_lane = ego_veh_lane - 1;
			if(target_lane < LEFT_MOST_LANE)
			{
				target_lane = LEFT_MOST_LANE;
			}
			
			if(target_lane != ego_veh_lane)
			{
				if(TRUE == IsOvertakePossible(vehicle_state,sensor_fusion,target_lane))
				{
					new_state = STATE_CHANGE_LANE_LEFT;
					overtake_target_lane = target_lane;
					break;
				}
			}
			
			// if left overtake is not possible, try right
			target_lane = ego_veh_lane + 1;
			if(target_lane > RIGHT_MOST_LANE)
			{
				target_lane = RIGHT_MOST_LANE;
			}
			
			if(target_lane != ego_veh_lane)
			{
				if(TRUE == IsOvertakePossible(vehicle_state,sensor_fusion,target_lane))
				{
					new_state = STATE_CHANGE_LANE_RIGHT;
					overtake_target_lane = target_lane;
				}
			}
			
			break;
		case STATE_CHANGE_LANE_LEFT:
		case STATE_CHANGE_LANE_RIGHT:
			if(fabs(vehicle_state.d - ((overtake_target_lane*LANE_WIDTH) + (LANE_WIDTH/2))) < 1.0)
			{
				new_state = STATE_KEEP_LANE;
			}
			break;
		default:
			break;
	}
	
	return new_state;
}

bool Path_Planner::IsOvertakePossible(struct vehicle_current_state vehicle_state,
									  const vector<vector<double>> &sensor_fusion,
									  short target_lane)
{
	bool is_possible = TRUE;
	short rear_min_dist;
	
	for(short i=0;i<sensor_fusion.size();i++)
	{
		if(target_lane == GetLaneNum(sensor_fusion[i][SENS_FUSION_CAR_D_POS]))
		{
			if((sensor_fusion[i][SENS_FUSION_CAR_S_POS] > vehicle_state.s) &&
			   (sensor_fusion[i][SENS_FUSION_CAR_S_POS] - vehicle_state.s < MIN_DIST_TO_OVERTAKE_FRONT))
			{
				double sens_fusion_speed = distance(0,0,sensor_fusion[i][SENS_FUSION_CAR_VEL_DX],sensor_fusion[i][SENS_FUSION_CAR_VEL_DY]);
					
				// if the ego vehicle is faster than the other vehicle or the delta velocity < 3mph
				if(vehicle_state.speed - sens_fusion_speed >= -3)
				{
					is_possible = FALSE;
				}
			}
			
			if(vehicle_state.speed >= 40)
			{
				rear_min_dist = MIN_DIST_TO_OVERTAKE_REAR_HIGH_SPEED;
			}
			else if(vehicle_state.speed >= 33)
			{
				rear_min_dist = MIN_DIST_TO_OVERTAKE_REAR_MID_SPEED;
			}
			else
			{
				rear_min_dist = MIN_DIST_TO_OVERTAKE_REAR_LOW_SPEED;
			}
			
			if((sensor_fusion[i][SENS_FUSION_CAR_S_POS] <= vehicle_state.s) &&
			   (vehicle_state.s - sensor_fusion[i][SENS_FUSION_CAR_S_POS] < rear_min_dist))
			{
				is_possible = FALSE;
			} 
		}
	}
	
	return is_possible;
}

short Path_Planner::GetLaneNum(double d)
{
	short lane_num;
	
	if(d < ((LEFT_MOST_LANE+1)*LANE_WIDTH))
	{
		lane_num = LEFT_MOST_LANE;
	}
	else if(d < ((MIDDLE_LANE+1)*LANE_WIDTH))
	{
		lane_num = MIDDLE_LANE;
	}
	else
	{
		lane_num = RIGHT_MOST_LANE;
	}
	
	return lane_num;
}