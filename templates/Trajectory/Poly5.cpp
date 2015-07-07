// Poly5.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "TrajData.h"
#include "Trajectory.h"

#define DATA TRAJ_DATA_Test2

int _tmain(int argc, _TCHAR* argv[])
{	int i = 0;
	FILE *out;
	out = fopen("poly5test.m", "w+");
	fprintf(out, "TestData = [...\n");
	fclose(out);

	int DATA_row = sizeof(DATA)/sizeof(DATA[0][0])/4;
	printf("rowDATA = %d\n", DATA_row);
	
	float T[] = {0.0, 2.0};
	int n = 200;
	float dt = (T[1]-T[0])/n;
	float t = 0;
	while(t<=T[1]){
		trajInterp(t, DATA, DATA_row);
		t = t + dt;
	}

	out = fopen("poly5test.m", "a");
	fprintf(out, "];\n");
	fprintf(out,"subplot(3,1,1);\n");
	fprintf(out, "plot(TestData(:,1),TestData(:,2));\n");
	fprintf(out,"subplot(3,1,2);\n");
	fprintf(out, "plot(TestData(:,1),TestData(:,3));\n");
	fprintf(out,"subplot(3,1,3);\n");
	fprintf(out, "plot(TestData(:,1),TestData(:,4));\n");
	fclose(out);

	while(1);
	return 0;
}


