#include "parameters.h"

// v_start, v_end, v_max, a_max
Trajectory::Parameters p_straight_start(0,300,300,2000);
Trajectory::Parameters p_straight(300,300,300,2000);
Trajectory::Parameters p_straight_acc(300,300,600,3000);
Trajectory::Parameters p_straight_end(300,0,300,2000);
Trajectory::Parameters p_turn(300,300,300,2000);

Trajectory::Parameters p_miniturn(0,0,120,100);
Trajectory::Parameters p_ministraight(0,0,300,2000);

Trajectory::Parameters p_faststraight_start(0,950,3000,12000);
Trajectory::Parameters p_faststraight(950,950,3000,12000);
Trajectory::Parameters p_faststraight_end(950,0,3000,12000);
Trajectory::Parameters p_fastturn(950,950,950,5000);

Trajectory::Parameters pa_faststraight_start[] = {
	Trajectory::Parameters(	0,	350,	600,	1500),
	Trajectory::Parameters(	0,	400,	700,	3000),
	Trajectory::Parameters(	0,	400,	800,	3000),
	Trajectory::Parameters(	0,	450,	900,	3000),
	Trajectory::Parameters(	0,	500,	1000,	4000),
	Trajectory::Parameters(	0,	550,	1200,	4000),
	Trajectory::Parameters(	0,	650,	1400,	4000)
};

Trajectory::Parameters pa_faststraight[] = {
	Trajectory::Parameters(	350,	350,	600,	1500),
	Trajectory::Parameters(	400,	400,	700,	3000),
	Trajectory::Parameters(	400,	400,	800,	3000),
	Trajectory::Parameters(	450,	450,	900,	3000),
	Trajectory::Parameters(	500,	500,	1000,	4000),
	Trajectory::Parameters(	550,	550,	1200,	4000),
	Trajectory::Parameters(	650,	650,	1400,	4000)
};

Trajectory::Parameters pa_faststraight_end[] = {
	Trajectory::Parameters(	350,	0,	350,	1500),
	Trajectory::Parameters(	400,	0,	400,	3000),
	Trajectory::Parameters(	400,	0,	400,	3000),
	Trajectory::Parameters(	450,	0,	450,	3000),
	Trajectory::Parameters(	500,	0,	500,	4000),
	Trajectory::Parameters(	550,	0,	550,	4000),
	Trajectory::Parameters(	650,	0,	650,	4000)
};

Trajectory::Parameters pa_fastturn[] = {
	Trajectory::Parameters(	350,	350,	600,	1500),
	Trajectory::Parameters(	400,	400,	700,	3000),
	Trajectory::Parameters(	400,	400,	800,	3000),
	Trajectory::Parameters(	450,	450,	900,	3000),
	Trajectory::Parameters(	500,	500,	1000,	4000),
	Trajectory::Parameters(	550,	550,	1200,	4000),
	Trajectory::Parameters(	650,	650,	1400,	4000)
};
