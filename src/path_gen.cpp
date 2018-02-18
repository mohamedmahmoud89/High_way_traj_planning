#include "path_gen.h"
#include <iostream>

#define MPH_TO_KPH 2.24

Path_Gen::Path_Gen(const vector<double> &maps_s_in, const vector<double> &maps_x_in, const vector<double> &maps_y_in)
{
	for(int i=0; i<maps_s_in.size();i++)
	{
		maps_s.push_back(maps_s_in[i]);
		maps_x.push_back(maps_x_in[i]);
		maps_y.push_back(maps_y_in[i]);
	}
}

vector<point> Path_Gen::GeneratePath(struct vehicle_current_state vehicle_state,
								short target_lane_num,
								double max_speed_mph,
								short overtake_smoothing_factor,
								const vector<double> &previous_path_x,
								const vector<double> &previous_path_y)
{
	point pt;
	vector<point> out_path;
	vector<double> tmp;
	vector<point> return_path;
	vector<double> ptsx;
	vector<double> ptsy;
	double ref_x,ref_y,ref_yaw;
	
	short prev_path_size = previous_path_x.size();
	
	if(prev_path_size >= 2)
	{
		ptsx.push_back(previous_path_x[previous_path_x.size() - 2]);
		ptsx.push_back(previous_path_x[previous_path_x.size() - 1]);
		ptsy.push_back(previous_path_y[previous_path_y.size() - 2]);
		ptsy.push_back(previous_path_y[previous_path_y.size() - 1]);
		ref_x = previous_path_x[previous_path_x.size() - 1];
		ref_y = previous_path_y[previous_path_y.size() - 1];
		ref_yaw = atan2(ref_y - previous_path_y[previous_path_y.size() - 2],
						ref_x - previous_path_x[previous_path_x.size() - 2]);
	}
	else
	{
		ptsx.push_back(vehicle_state.x - cos(vehicle_state.yaw));
		ptsx.push_back(vehicle_state.x);
		ptsy.push_back(vehicle_state.y - sin(vehicle_state.yaw));
		ptsy.push_back(vehicle_state.y);
		ref_x = vehicle_state.x;
		ref_y = vehicle_state.y;
		ref_yaw = deg2rad(vehicle_state.yaw);
	}
	
	for(short i=1;i<=3;i++)
	{
		tmp = getXY(vehicle_state.s+(30*i*overtake_smoothing_factor),2+4*target_lane_num,maps_s,maps_x,maps_y);
		ptsx.push_back(tmp[0]);
		ptsy.push_back(tmp[1]);
	}
	
	for(short i=0;i<ptsx.size();i++)
	{
		/*cout << "ptsx_before = " << ptsx[i] << endl;
		cout << "ptsy_before = " << ptsy[i] << endl;*/
		
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;
		
		ptsx[i] = (shift_x*cos(0-ref_yaw)) - (shift_y*sin(0-ref_yaw));
		ptsy[i] = (shift_x*sin(0-ref_yaw)) + (shift_y*cos(0-ref_yaw));
		
		/*cout << "ptsx_after = " << ptsx[i] << endl;
		cout << "ptsy_after = " << ptsy[i] << endl;*/
	}
	
	s.set_points(ptsx,ptsy);
	
	for(short i=0;i<prev_path_size;i++)
	{
		pt.x = previous_path_x[i];
		pt.y = previous_path_y[i];
		out_path.push_back(pt);
	}
	
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = distance(0,0,target_x,target_y);
	double x_add_on = 0;
	double N = target_dist/(0.02*max_speed_mph/MPH_TO_KPH);
	
	for(short i=0;i<50-prev_path_size;i++)
	{
		double x_point = x_add_on + (target_dist/N);
		double y_point = s(x_point);
		x_add_on = x_point;
		
		double temp_x = x_point;
		double temp_y = y_point;
		
		x_point = temp_x*cos(ref_yaw) - temp_y*sin(ref_yaw);
		y_point = temp_x*sin(ref_yaw) + temp_y*cos(ref_yaw);
		
		x_point += ref_x;
		y_point += ref_y;
		
		pt.x = x_point;
		pt.y = y_point;
		out_path.push_back(pt);
	}
	
	return out_path;
}								
