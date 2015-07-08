#ifndef __TRAJDATA_H__
#define __TRAJDATA_H__

//input data are formatted as follows 
//1st col: time 
//2nd col: position
//3rd col: slope
//4th col: curvature 

static const float TRAJ_DATA_Test0[2][4] = {{    0,    0,  0.7,  0.1}, 
											{  2.5,    0,  0.7,  0.1}	
										   };

static const float TRAJ_DATA_Test1[3][4] = {{    0,  0.1, -0.5,  0.1}, 
											{	 1,    1,    0,   -2},
											{    2,  0.1, -0.5,  0.1}	
										   };

static const float TRAJ_DATA_Test2[6][4] = {{    0,  0.1, -0.5,  0.1}, 
											{  0.5,    1,    0,   -2},
											{    2,  0.1, -0.5,  0.1},
											{  2.8,  0.1, -0.5,  0.1}, 
											{  3.5,    1,    0,   -2},
											{	 5,  0.1, -0.5,  0.1}
										   };

#endif  // __TRAJDATA_H__

