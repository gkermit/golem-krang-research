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
bool getLiberty(Eigen::VectorXd& config, Eigen::VectorXd& config2, Eigen::VectorXd& config3, Eigen::VectorXd& config4) {

	// Get the data
	int r = 0;
	config.setZero(); 
	Somatic__Liberty *l_msg = SOMATIC_GET_LAST_UNPACK( r, somatic__liberty,
													   &protobuf_c_system_allocator, 1024, &achChannel);
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (l_msg == NULL)) return false;

	// Set the values for the position
	Somatic__Vector* s1 = l_msg->sensor1;
	Somatic__Vector* s2 = l_msg->sensor2;
	Somatic__Vector* s3 = l_msg->sensor3;
	Somatic__Vector* s4 = l_msg->sensor4;

	config << s1->data[0], -s1->data[1], -s1->data[2], 0.0, 0.0, 0.0;
	config2 << s2->data[0], -s2->data[1], -s2->data[2], 0.0, 0.0, 0.0;
	config3 << s3->data[0], -s3->data[1], -s3->data[2], 0.0, 0.0, 0.0;
	config4 << s4->data[0], -s4->data[1], -s4->data[2], 0.0, 0.0, 0.0;

	// Convert from a quaternion to rpy representation

	//s1
	Eigen::Quaternion <double> oriQ (s1->data[6], s1->data[3], s1->data[4], s1->data[5]);
	Eigen::Matrix3d oriM = oriQ.matrix();
	Eigen::Vector3d oriE = matrixToEuler(oriM);
	config.bottomLeftCorner<3,1>() << -oriE[2], -oriE[1], oriE[0];

	//s2
	Eigen::Quaternion <double> oriQ2 (s2->data[6], s2->data[3], s2->data[4], s2->data[5]);
	Eigen::Matrix3d oriM2 = oriQ2.matrix();
	Eigen::Vector3d oriE2 = matrixToEuler(oriM2);
	config2.bottomLeftCorner<3,1>() << -oriE2[2], -oriE2[1], oriE2[0];

	//s3
	Eigen::Quaternion <double> oriQ3 (s3->data[6], s3->data[3], s3->data[4], s3->data[5]);
	Eigen::Matrix3d oriM3 = oriQ3.matrix();
	Eigen::Vector3d oriE3 = matrixToEuler(oriM3);
	config3.bottomLeftCorner<3,1>() << -oriE3[2], -oriE3[1], oriE3[0];

	//s4
	Eigen::Quaternion <double> oriQ4 (s4->data[6], s4->data[3], s4->data[4], s4->data[5]);
	Eigen::Matrix3d oriM4 = oriQ4.matrix();
	Eigen::Vector3d oriE4 = matrixToEuler(oriM4);
	config4.bottomLeftCorner<3,1>() << -oriE4[2], -oriE4[1], oriE4[0];


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
		Eigen::VectorXd config2 = VectorXd::Zero(6);
		Eigen::VectorXd config3 = VectorXd::Zero(6);
		Eigen::VectorXd config4 = VectorXd::Zero(6);
		bool success = false;
		while(!success) success = getLiberty(config, config2, config3, config4);
		cout << "config: " << config.transpose() << endl;
		
		//sensor 1 (palm)
		Eigen::Matrix3d matrix = (Eigen::AngleAxis <double> (config(5), Eigen::Vector3d(0.0, 0.0, 1.0)) *
					Eigen::AngleAxis <double> (config(4), Eigen::Vector3d(0.0, 1.0, 0.0)) *
					Eigen::AngleAxis <double> (config(3), Eigen::Vector3d(1.0, 0.0, 0.0))).matrix();
		cout << "matrix: \n" << matrix << "\n" << endl;
	
		//sensor 2 (finger 1)
		Eigen::Matrix3d matrix2 = (Eigen::AngleAxis <double> (config2(5), Eigen::Vector3d(0.0, 0.0, 1.0)) *
					Eigen::AngleAxis <double> (config2(4), Eigen::Vector3d(0.0, 1.0, 0.0)) *
					Eigen::AngleAxis <double> (config2(3), Eigen::Vector3d(1.0, 0.0, 0.0))).matrix();
		cout << "matrix2: \n" << matrix2 << "\n" << endl;

		//sensor 3 (finger 2)
		Eigen::Matrix3d matrix3 = (Eigen::AngleAxis <double> (config3(5), Eigen::Vector3d(0.0, 0.0, 1.0)) *
					Eigen::AngleAxis <double> (config3(4), Eigen::Vector3d(0.0, 1.0, 0.0)) *
					Eigen::AngleAxis <double> (config3(3), Eigen::Vector3d(1.0, 0.0, 0.0))).matrix();
		cout << "matrix3: \n" << matrix3 << "\n" << endl;


		//sensor 4 (finger 3)
		Eigen::Matrix3d matrix4 = (Eigen::AngleAxis <double> (config4(5), Eigen::Vector3d(0.0, 0.0, 1.0)) *
					Eigen::AngleAxis <double> (config4(4), Eigen::Vector3d(0.0, 1.0, 0.0)) *
					Eigen::AngleAxis <double> (config4(3), Eigen::Vector3d(1.0, 0.0, 0.0))).matrix();
		cout << "matrix4: \n" << matrix4 << "\n" << endl;


		//angle of sensor 1 relative to polhemus cube
		Eigen::Vector3d localZ (matrix(0,2), matrix(1,2), matrix(2,2));
		double angle = acos(localZ.dot(Eigen::Vector3d(0.0, 0.0, 1.0)));
		cout << "angle1: " << angle / M_PI * 180.0 << endl;

		//angle of sensor 1 relative to sensor 2, 3
		Eigen::Vector3d sensor2Z (matrix2(0,2), matrix2(1,2), matrix2(2,2));
		double angle2 = acos(sensor2Z.dot(Eigen::Vector3d(-matrix(0,2), -matrix(1,2), -matrix(2,2))));
		cout << "angle2: " << angle2 / M_PI * 180.0 << endl;
		//angle of sensor 1 relative to sensor 2, 3
		Eigen::Vector3d sensor3Z (matrix3(0,2), matrix3(1,2), matrix3(2,2));
		double angle3 = acos(sensor3Z.dot(Eigen::Vector3d(-matrix(0,2), -matrix(1,2), -matrix(2,2))));
		cout << "angle3: " << angle3 / M_PI * 180.0 << endl;

		//angle of sensor 1 relative to sensor 2, 3
		Eigen::Vector3d sensor4Z (matrix4(0,2), matrix4(1,2), matrix4(2,2));
		double angle4 = acos(sensor4Z.dot(Eigen::Vector3d(-matrix(0,2), -matrix(1,2), -matrix(2,2))));
		cout << "angle4: " << angle4 / M_PI * 180.0 << endl;

		usleep(1e5);
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
