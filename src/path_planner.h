#include <vector>
#include "path_gen.h"

using namespace std;

class Path_Planner
{
	public:
	//functions
	Path_Planner(const vector<double> &maps_s_in, const vector<double> &maps_x_in, const vector<double> &maps_y_in,short path_size_in);
	vector<point> PathUpdate(struct vehicle_current_state vehicle_state,
							const vector<double> &previous_path_x,
							const vector<double> &previous_path_y,
							const vector<vector<double>> &sensor_fusion);
							
	private:
	//types
	enum path_planning_state
	{
		STATE_START_UP = 0,
		STATE_KEEP_LANE,
		STATE_PREPARE_CHANGE_LANE,
		STATE_CHANGE_LANE_LEFT,
		STATE_CHANGE_LANE_RIGHT
	};
	
	//variables
	vector<double> maps_s;
	vector<double> maps_x;
	vector<double> maps_y;
	vector<double> path_x_pts;
	vector<double> path_y_pts;
	short path_size;
	enum path_planning_state current_state;
	Path_Gen *p_path_generator;
	short id_of_vehicle_ahead;
	short overtake_target_lane;
	
	//functions
	enum path_planning_state UpdateStateMAchine(struct vehicle_current_state vehicle_state,
												const vector<vector<double>> &sensor_fusion);
	short GetLaneNum(double d);
	bool IsOvertakePossible(struct vehicle_current_state vehicle_state,
							const vector<vector<double>> &sensor_fusion,
							short target_lane);
};