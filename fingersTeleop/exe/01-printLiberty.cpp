/**
 * @file 01-printLiberty.cpp
 * @author Can Erdogan, Greg Tracy
 * @date Sept 21, 2013
 * @brief This executable shows how to get and print the liberty data reading 
 * from the "liberty" ach channel.
 */

#include <Eigen/Dense>
#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;
ach_channel_t achChannel;
const char *channelName = "liberty";

using namespace Eigen;
using namespace std;
#define M_EPSILON 1e-10

/* ********************************************************************************************* */
Eigen::Vector3d matrixToEuler(Matrix3d& m) {

	double x, y, z;
	if(m(2, 0) > (1.0-M_EPSILON)) {
		x = atan2(m(0, 1), m(0, 2));
		y = -M_PI / 2.0;
		z = 0.0;
	}   
	if(m(2, 0) < -(1.0-M_EPSILON)) {
		x = atan2(m(0, 1), m(0, 2));
		y = M_PI / 2.0;
		z = 0.0;
	}   
	x = atan2(m(2, 1), m(2, 2));
	y = -asin(m(2, 0));
	z = atan2(m(1, 0), m(0, 0));
	return Vector3d(x,y,z); 
}   

/* ********************************************************************************************* */
bool getLiberty(Eigen::VectorXd& config) {

	// Get the data
	int r = 0;
	config.setZero(); 
	Somatic__Liberty *l_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__liberty,
													   &protobuf_c_system_allocator, 1024, &achChannel);
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (l_msg == NULL)) return false;

	// Set the values for the position
	Somatic__Vector* s1 = l_msg->sensor1;
	config << s1->data[0], -s1->data[1], -s1->data[2], 0.0, 0.0, 0.0;

	// Convert from a quaternion to rpy representation
	Eigen::Quaternion <double> oriQ (s1->data[6], s1->data[3], s1->data[4], s1->data[5]);
	Eigen::Matrix3d oriM = oriQ.matrix();
	Eigen::Vector3d oriE = matrixToEuler(oriM);
	config.bottomLeftCorner<3,1>() << -oriE[2], -oriE[1], oriE[0];
	return true;
}

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Get the liberty data
		Eigen::VectorXd config = VectorXd::Zero(6);
		bool success = false;
		while(!success) success = getLiberty(config);
		cout << config.transpose() << endl;

		// Free buffers allocated during this cycle
		aa_mem_region_release(&somaticContext.memreg);	
	}

	// Send the stoppig event
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void init() {
	somatic_d_init(&somaticContext, &somaticOptions);
	somatic_d_channel_open(&somaticContext, &achChannel, channelName, NULL);
}

/* ********************************************************************************************* */
void destroy() {
	somatic_d_channel_close(&somaticContext, &achChannel);
	somatic_d_destroy(&somaticContext);
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

	// Set the somatic context options
	somaticOptions.ident = "01-libertyPrint";
	somaticOptions.sched_rt = SOMATIC_D_SCHED_NONE; 
	somaticOptions.skip_mlock = 1; 		

	// Initialize the code and run until a somatic_sig is received (?)
	init();
	run();
	destroy();

	exit(EXIT_SUCCESS);
}
