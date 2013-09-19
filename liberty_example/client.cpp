/** 
 * @date Sept 17, 2013
 * @brief This file shows an example of how to send a
 * Liberty message using ach and somatic.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

#include <iostream>

using namespace std;

/// argp program version
const char *argp_program_version = "client 0.0";
#define ARGP_DESC "writes somatic events to syslog"

// The somatic context and options
somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;

// The ach channel and its name
ach_channel_t achChannel;
const char *channelName;

// The Liberty message which is initialized in init() and filled out in run()
Somatic__Liberty* libertyMessage;

/* ********************************************************************************************* */
void init() {

	somatic_opt_verbosity = 9;

	// Initialize the message
	libertyMessage = somatic_liberty_alloc();
	libertyMessage->meta = somatic_metadata_alloc();
	libertyMessage->meta->type = SOMATIC__MSG_TYPE__LIBERTY;
	libertyMessage->meta->has_type = 1;

	// Initialize the somatic context 
	somatic_d_init(&somaticContext, &somaticOptions);
	somatic_d_channel_open(&somaticContext, &achChannel, channelName, NULL);
}

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	double* x = new double [24];
	while(!somatic_sig_received) { 

		// get data
		for(size_t i = 0; i < 24; i++) x[i] = ((double) rand()) / RAND_MAX;
		somatic_verbprintf(1, "Liberty:\n%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\n%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\n%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\n%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\n",
				x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11], x[12], x[13], x[14], x[15], x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23]);

		// fill message

		aa_fcpy( libertyMessage->sensor1->data, x, 6 );
		aa_fcpy( libertyMessage->sensor2->data, x+6, 6 );
		aa_fcpy( libertyMessage->sensor3->data, x+12, 6 );
    		aa_fcpy( libertyMessage->sensor4->data, x+18, 6 );
 
		somatic_metadata_set_time_now(libertyMessage->meta);
		somatic_metadata_set_until_duration( libertyMessage->meta, .1);

		// send message
		ach_status_t result = SOMATIC_PACK_SEND(&achChannel, somatic__liberty, libertyMessage);
		if(ACH_OK != result)
			fprintf(stderr, "Couldn't send message: %s\n", ach_result_to_string(result));
	}   

	// Send the stoppig event
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {

	// Close the channel and end the daemon
	somatic_d_channel_close(&somaticContext, &achChannel);
	somatic_d_destroy(&somaticContext);
}

/* ********************************************************************************************* */
int main() {

	// Set the somatic context options
	somaticOptions.ident = "client";
	somaticOptions.sched_rt = SOMATIC_D_SCHED_NONE; // logger not realtime
	somaticOptions.skip_mlock = 0; // logger not realtime, other daemons may be

	// Set the channel name
	channelName = "chan_liberty";

	init();
	run();
	destroy();

	return 0;
}
