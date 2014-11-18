#include "CCalibBall.h"

int main(int argc, char **argv) {


	char config_file[100];
	if (argc == 1)
		strcpy(config_file, "config.yml");
	else strcpy(config_file, argv[1]);
	CCalibBall calib(config_file);
	calib.run(config_file);
	return 0;
}