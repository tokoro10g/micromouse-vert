#include "parameters.h"

// v_start, v_end, v_max, a_max
Trajectory::Parameters p_straight_start(0,300,300,2000);
Trajectory::Parameters p_straight(300,300,300,2000);
Trajectory::Parameters p_straight_acc(300,300,1000,3000);
Trajectory::Parameters p_straight_end(300,0,300,2000);
Trajectory::Parameters p_turn(300,300,300,2000);

Trajectory::Parameters p_miniturn(0,0,80,80);
Trajectory::Parameters p_ministraight(0,0,800,5000);

Trajectory::Parameters p_faststraight_start(0,950,3000,12000);
Trajectory::Parameters p_faststraight(950,950,3000,12000);
Trajectory::Parameters p_faststraight_to_endturn(950,850,3000,12000);
Trajectory::Parameters p_faststraight_end(950,0,3000,12000);

Trajectory::Parameters p_faststraight_max_start(0,3000,3000,12000);
Trajectory::Parameters p_faststraight_max(3000,3000,3000,12000);
Trajectory::Parameters p_faststraight_max_from_turn(950,3000,3000,12000);
Trajectory::Parameters p_faststraight_max_to_turn(3000,950,3000,12000);
Trajectory::Parameters p_faststraight_max_end(3000,0,3000,12000);

Trajectory::Parameters p_fastturn(950,950,950,5000);
Trajectory::Parameters p_fastendturn(850,850,950,5000);
Trajectory::Parameters p_faststraight_startend(0,0,3000,12000);

Trajectory::Parameters pa_faststraight_start[] = {
	Trajectory::Parameters(	0,	350,	600,	1500),
	Trajectory::Parameters(	0,	800,	1500,	5000),
	Trajectory::Parameters(	0,	1000,	2000,	5000),
	Trajectory::Parameters(	0,	1000,	2200,	5000),
	Trajectory::Parameters(	0,	1000,	2500,	6000),
	Trajectory::Parameters(	0,	1050,	3000,	8000),
	Trajectory::Parameters(	0,	1050,	3000,	10000)
};

Trajectory::Parameters pa_faststraight[] = {
	Trajectory::Parameters(	350,	350,	600,	1500),
	Trajectory::Parameters(	800,	800,	1500,	5000),
	Trajectory::Parameters(	1000,	1000,	2000,	5000),
	Trajectory::Parameters(	1000,	1000,	2200,	5000),
	Trajectory::Parameters(	1000,	1000,	2500,	6000),
	Trajectory::Parameters(	1050,	1050,	3000,	8000),
	Trajectory::Parameters(	1050,	1050,	3000,	10000)
};

Trajectory::Parameters pa_faststraight_end[] = {
	Trajectory::Parameters(	350,	0,	350,	1500),
	Trajectory::Parameters(	800,	0,	800,	5000),
	Trajectory::Parameters(	1000,	0,	1000,	5000),
	Trajectory::Parameters(	1000,	0,	1000,	5000),
	Trajectory::Parameters(	1000,	0,	1000,	6000),
	Trajectory::Parameters(	1050,	0,	1050,	8000),
	Trajectory::Parameters(	1050,	0,	1050,	9000)
};

Trajectory::Parameters pa_fastturn[] = {
	Trajectory::Parameters(	350,	350,	600,	1500),
	Trajectory::Parameters(	800,	800,	1500,	5000),
	Trajectory::Parameters(	1000,	1000,	2000,	5000),
	Trajectory::Parameters(	1000,	1000,	2200,	5000),
	Trajectory::Parameters(	1000,	1000,	2500,	6000),
	Trajectory::Parameters(	1050,	1050,	3000,	8000),
	Trajectory::Parameters(	1050,	1050,	3000,	10000)
};
