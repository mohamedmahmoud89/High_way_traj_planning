#include <vector>
#include "tools.h"
#include "spline.h"

using namespace std;
using namespace tk;

class Path_Gen
{
	public:
	//functions
	Path_Gen(const vector<double> &maps_s_in, const vector<double> &maps_x_in, const vector<double> &maps_y_in);
	vector<point> GeneratePath(struct vehicle_current_state vehicle_state,
								short target_lane_num,
								double max_speed_mph,
								short overtake_smoothing_factor,
								const vector<double> &previous_path_x,
							    const vector<double> &previous_path_y);
								
	private:
	//variables
	vector<double> maps_s;
	vector<double> maps_x;
	vector<double> maps_y;
	spline s;
};