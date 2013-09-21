/**
 * @date Sept 17, 2013
 * @brief This file shows an example usage of the somatic library. The server
 * creates an ach channnel and processes the messages received on it.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

/// argp program version
const char *argp_program_version = "server 0.0";
#define ARGP_DESC "writes somatic events to syslog"

// The somatic context and options
somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;

// The ach channel and its name
ach_channel_t achChannel;
const char *channelName;

// Argument processing
static int parse_opt( int key, char *arg, struct argp_state *state);
extern struct argp_option argp_options[];
extern struct argp argp;

/* ********************************************************************************************* */
void init() {

	somatic_opt_verbosity = 9;

	// =======================================================
	// A. Create an ach channel
	
	// Create the attributes
	ach_create_attr_t attr;
	ach_create_attr_init(&attr);

	// Create the channel
	const size_t messageCount = 512;
	const size_t messageSize = ACH_DEFAULT_FRAME_SIZE;
	ach_status_t result = ach_create(channelName, messageCount, messageSize, &attr);
	if(result != ACH_OK) {
		fprintf(stderr, "Error creating channel %s: %s\n", channelName, ach_result_to_string(result));
		exit(EXIT_FAILURE); 
	}

	// =======================================================
	// B. Change the channel mode
	// NOTE: To do that, we need to open the channel, set the option and then close it.

	// Open the channel
	ach_channel_t chan;
	result = ach_open(&chan, channelName, NULL);
	if(result != ACH_OK) {
		fprintf(stderr, "Couldn't open channel for chmod: %s\n", ach_result_to_string(result));
		exit(EXIT_FAILURE); 
	}

	// Change the mode
	mode_t mode = 666;
	result = ach_chmod(&chan, mode);
	if(result != ACH_OK) {
		fprintf(stderr, "Couldn't chmod: %s%s", ach_result_to_string(result), 
				(result == ACH_FAILED_SYSCALL) ? strerror(errno) : "\n");
		exit(EXIT_FAILURE); 
	}

	// Close the channel
	result = ach_close(&chan);
	if(result != ACH_OK) {
		fprintf(stderr, "Couldn't close channel after chmod: %s\n", ach_result_to_string(result));
		exit(EXIT_FAILURE); 
	}

	// =======================================================
	// C. Prepare the somatic context

	// Initialize the somatic context 
	// NOTE: somatic_d_init sends a SOMATIC__EVENT__CODES__PROC_STARTING message on the ach channel.
	// NOTE: somatic_d_channel opens the channel back again. Maybe we did not need to close it!
	somatic_d_init(&somaticContext, &somaticOptions);
	somatic_d_channel_open(&somaticContext, &achChannel, channelName, NULL);
}

/* ********************************************************************************************* */
void update() {

	// =======================================================
	// A. Get message
	// NOTE: This is usually done with SOMATIC_D_GET which is a macro.

	// Set the time to read (?)
	struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	uint8_t* buffer = (uint8_t*) somatic_d_get(&somaticContext, &achChannel, &numBytes, &abstimeout, ACH_O_WAIT, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return;

	// =======================================================
	// B. Read message

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(somaticContext.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return;
	if(msg->meta->type != SOMATIC__MSG_TYPE__LIBERTY) return;

	// Read the liberty message
	Somatic__Liberty* libertyMessage = somatic__liberty__unpack(&(somaticContext.pballoc), numBytes, buffer);
	printf("[server] Liberty:\n");

	for(size_t i = 0; i < 6; i++)
        	printf("%6.2f  ", libertyMessage->sensor1->data[i]);
	printf("\n");
	for(size_t i = 0; i < 6; i++)
                printf("%6.2f  ", libertyMessage->sensor2->data[i]);
	printf("\n");
	for(size_t i = 0; i < 6; i++)
                printf("%6.2f  ", libertyMessage->sensor3->data[i]);
	printf("\n");
	for(size_t i = 0; i < 6; i++)
                printf("%6.2f  ", libertyMessage->sensor4->data[i]);
	printf("\n"); fflush(stdout);
}

/* ********************************************************************************************* */
void run() {

	// Send a message; set the event code and the priority
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {
		update();
		aa_mem_region_release(&somaticContext.memreg);	// free buffers allocated during this cycle
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
	somaticOptions.ident = "server";
	somaticOptions.sched_rt = SOMATIC_D_SCHED_NONE; // logger not realtime
	somaticOptions.skip_mlock = 0; // logger not realtime, other daemons may be

	// Set the channel name
	channelName = "chan_liberty";

	init();
	run();
	destroy();

	return 0;
}
