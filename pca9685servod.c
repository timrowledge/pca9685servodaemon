/* PCA9685servoblaster
 * tim@rowledge.org
 * Originally inspired by
 * https://github.com/richardghirst/PiBits/tree/master/ServoBlaster
 * and fixed, rewritten, reworked and finally rewritten again to eventually work
 *
 * Released under the MIT license
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>

#include <pigpiod_if2.h>

// uncomment this next line if you want a lot of debug output
// #define DEBUG 1

#ifndef  DEBUG
# define DEBUG	0
#endif


#if (DEBUG)
void __sq_DPRINTF(const char *fmt, ...);
# define DPRINTF(ARGS) __sq_DPRINTF ARGS
#else
# define DPRINTF(ARGS)	((void)0)
#endif


void 
__sq_DPRINTF(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vfprintf(stderr,fmt, ap);
  va_end(ap);
}


#define DEFAULT_cycleTimeUSec	1000000/50 // default to 50Hz -> 20,000uS
#define MIN_cycleTimeUSec	655
#define MAX_cycleTimeUSec	41666
#define DEFAULT_stepTimeUSec	5 // gives 400 steps for typical servo range
#define DEFAULT_servoMinPulseUSec	500 // Be aware that many cheap servos get very annoyed
#define DEFAULT_servoMaxPulseUSec	2500// by getting pushed too far. Use the min/max
                                    // options to change the limits

#define PCADEVICEFILE			"/dev/pca9685servo"
#define MAX_SERVOS	16
#define I2C_BUS 1
#define DEFAULT_PCA_ADDR 0x40	

// PCA9685 Register & Mode Definitions

#define MODE1 0x00			//Mode  register  1
#define MODE2 0x01			//Mode  register  2
#define SUBADR1 0x02		//I2C-bus subaddress 1
#define SUBADR2 0x03		//I2C-bus subaddress 2
#define SUBADR3 0x04		//I2C-bus subaddress 3
#define ALLCALLADR 0x05     //LED All Call I2C-bus address
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels
#define ALLLED_ON_L 0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB	//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC	//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD	//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE		//prescaler for output frequency
#define CLOCK_FREQ 25000000.0 //25MHz default osc clock
#define BUFFER_SIZE 0x08  //1 byte buffer
// MODE1 reg flags
#define RESTART 0x80
#define EXTCLK 0x40
#define AI 0x20
#define SLEEP 0x10
#define SUB1 0x8
#define SUB2 0x4
#define SUB3 0x2
#define ALLCALL 0x1
// MODE2 reg flags
#define INVRT 0x10
#define OCH 0x8
#define OUTDRV 0x4
#define OUTNE // doesn't matter here

// pigpio values
static int pi= -1, pca= -1; // the pigpiod handles, initialised to less than 0

// device address
unsigned int i2c_address;

// cycleTimeUSec is the pulse cycle time per servo, in microseconds.
// Typically it should be 20ms for a 50Hz frame; it gets adjusted to match the
// actual value achieved by the PCA9685
// stepTimeUSec is the pulse width increment granularity, again in microseconds.

static double cycleTimeUSec;
static double stepTimeUSec;
static uint8_t timer_prescale; // timer setting byte for the PCA9685

static int servoStart[MAX_SERVOS];
static double servoWidth[MAX_SERVOS];
static double servoMinPulseUSec, servoMaxPulseUSec;
#if (DEBUG)
    unsigned int on_val, off_val;
#endif
	
static void terminate(int dummy)
{
    /* disconnect from pigpiod and release the file descriptors */
	unlink(PCADEVICEFILE);
	if ( pi >= 0) {
    	i2c_close(pi, pca);
    	pigpio_stop(pi);
    }
	exit(1);
}

static void fatal(char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	terminate(0);
}

#if (DEBUG)
static void read_servo(int servo, unsigned int *on, unsigned int *off) {
    unsigned int on_off[4];
    int i, p, ret;
    // read the pwm parameters for channel servo and return the raw 12 bit unsigned values
    for (i = LED0_ON_L, p = 0; i <= LED0_OFF_H; i++, p++ ) {
        ret = i2c_read_byte_data(pi, pca, i + LED_MULTIPLYER * servo);
        if (ret <0) {
            DPRINTF(("Bad i2c byte read for servo: %d\n", servo));
            *on = ret;
            *off = 0xFFFFFFFF;
            return;
        }
    }
    *on = on_off[0] | (on_off[1] <<8);
    *off = on_off[2] | (on_off[3] <<8);
}
#endif

static void  all_pwm_off(void)
{
    // turn off all pwm outputs
    i2c_write_byte_data(pi, pca, ALLLED_ON_L, 0);
    i2c_write_byte_data(pi, pca, ALLLED_ON_H, 0);
    i2c_write_byte_data(pi, pca, ALLLED_OFF_L, 0);
    i2c_write_byte_data(pi, pca, ALLLED_OFF_H, 0);
}

static void set_servo(int servo, double width)
{
    int i, p, ret;
	servoWidth[servo] = width;
    int onValue, offValue;
    unsigned int on_off[4];
    // set this servo to start at the servoStart tick and stay on for width ticks
    DPRINTF(( "set servo[%d]=%f %%\n", servo, width * 100.0));

    onValue = servoStart[servo];
    offValue = (onValue 
                + (int)((
                (servoWidth[servo] *  (servoMaxPulseUSec - servoMinPulseUSec) + servoMinPulseUSec)) / cycleTimeUSec 
                * 4096)
               )  % 4096;
    DPRINTF(( "pi: %d pca: %d servo: %d on: %d off: %d\n", pi, pca, servo, onValue, offValue));
    on_off[0] = onValue & 0xFF;   
    on_off[1] = onValue >> 8;   
    on_off[2] = offValue & 0xFF;   
    on_off[3] = offValue >> 8;   

    for (i = LED0_ON_L, p = 0; i <= LED0_OFF_H; i++, p++ ) {
        ret = i2c_write_byte_data(pi, pca, i + LED_MULTIPLYER * servo, on_off[p]);
        if (ret < 0) {
            DPRINTF(("Bad i2c byte[%d] write for servo: %d\n", p, servo));
            return;
        }
    }

#if (DEBUG)
    read_servo(servo, &on_val, &off_val);
    DPRINTF(("PCA actually registered on = %u off= %u\n", on_val, off_val));
    if (off_val == 0xFFFFFFFF) {
        DPRINTF(("Bad i2c read. errnum = %d\n", on_val));
    }
#endif
}

static void setup_sighandlers(void)
{
	int i;

	// Catch all signals possible
	for (i = 0; i < 64; i++) {
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}
}


static void init_servo_starts(int spreadout) {
    int servo, currentStart = 0;
	/* set the servo start ticks either at the same time (spreadout = 0) or
	to spread out the current draw . The former mighth help when driving LEDs */
	for (servo = 0; servo < MAX_SERVOS; servo++) {
		servoStart[servo] = currentStart;
		if (spreadout) currentStart += 4096 / MAX_SERVOS;
	}
}

static void calculateTimerSettings(double cycleTime) {
    double freq = 1.0e6 / cycleTime;
	timer_prescale = (CLOCK_FREQ / 4096 / freq)  - 1;
	DPRINTF(("Setting prescale value to: %d\n", timer_prescale));
	DPRINTF(("Actual frequency: %8.3f\n", (CLOCK_FREQ / 4096.0) / (timer_prescale + 1)));
	// adjust the global cycle time to reflect reality
	cycleTimeUSec = 1e6 * (timer_prescale +1)/ (CLOCK_FREQ / 4096.0);
	DPRINTF(("Actual cycle time: %8.3fus\n", cycleTimeUSec));
}

static void setPWMFreq(void) {
 	uint8_t oldmode, newmode;
 	int ret;
 	
 	ret = i2c_read_byte_data(pi, pca, MODE1);
 	oldmode = (uint8_t)ret;
    newmode = (oldmode & ~SLEEP) | SLEEP;    //sleep
	DPRINTF(("Setting prescale value to: %d\n", timer_prescale));
    i2c_write_byte_data(pi, pca, MODE1, newmode);        // go to sleep
    i2c_write_byte_data(pi, pca, PRE_SCALE, timer_prescale);
    i2c_write_byte_data(pi, pca, MODE1, oldmode);
    usleep(1000);
    i2c_write_byte_data(pi, pca, MODE1, oldmode | RESTART);
}

static void init_hardware(void) {
    uint8_t oldmode;
    int ret;
    // connect to the daemon, quit if that fails
    pi = pigpio_start(NULL, NULL);
    if (pi < 0)
	fatal("Unable to connect to pigpiod; is it running?\n");
    DPRINTF(("pigpio handle = %d\n", pi));

    // connect to the PCA9685 via i2c, quit if that fails
    // Consider having the i2c address an argument when starting the daemon?
    pca = i2c_open(pi, I2C_BUS, i2c_address, 0);
    if (pca  < 0)
        fatal("Unable to connect to PCA9685 hardware\n");
    DPRINTF(("pca handle = %d\n", pca));
    
    // initialise the PCA; write config byte to reg 0
    // See PCA9685.pdf 7.3.1
    // exactly what is best here is a bit arguable. I see 0x20 or 0x21 or 0 used variously
    
    all_pwm_off();
    ret = i2c_write_byte_data(pi, pca, MODE1, /* AI | */ ALLCALL);
    DPRINTF(("init_hardware MODE1 set = %d\n", ret));
    
    // maybe we should set some flags in MODE2 as well?
    // 0xC is used in at least one python based driver
     ret = i2c_write_byte_data(pi, pca, MODE2, /* OCH | */ OUTDRV );
    DPRINTF(("init_hardware MODE2 set %d\n", ret));
     // we have to wait for at least 500uS after setting the SLEEP flag to 0
    usleep(10000);
    ret = i2c_read_byte_data(pi, pca, MODE1);
    oldmode = (uint8_t)ret;
    
    ret = i2c_write_byte_data(pi, pca, MODE1, (oldmode & ~SLEEP));
    usleep(10000);
 
    setPWMFreq();
}

static double parse_width(int servo, char *width_arg) {
	char *p;
	char *digits = width_arg;
	double width;

    // step over any + or - modifier
	if (*width_arg == '-' || *width_arg == '+') {
		digits++;
	}

    // make sure the following string at least starts as a number
	if (*digits < '0' || *digits > '9') {
		return -1;
	}
	// read the actual float number
	width = strtod(digits, &p);
	DPRINTF(( "Raw width input = %f\n", width));

	if (*p == '\0') {
		// Specified in steps
		DPRINTF(( "steps specified -> %f\n", width));
        if (*width_arg == '+') {
            double current = (servoWidth[servo] *  (servoMaxPulseUSec - servoMinPulseUSec) + servoMinPulseUSec) / stepTimeUSec;
            DPRINTF(( "Add %f to %f = %f\n", width, current, current + width));
            width = current + width;
        } else if (*width_arg == '-') {
            double current = (servoWidth[servo] *  (servoMaxPulseUSec - servoMinPulseUSec) + servoMinPulseUSec) / stepTimeUSec;
            DPRINTF(( "Subtract %f from %f = %f\n", width, current, current - width));
            width = current - width;
        }
        // convert to fraction
        width = (width * stepTimeUSec - servoMinPulseUSec) / (servoMaxPulseUSec - servoMinPulseUSec);
	} else if (!strcmp(p, "us")) {
	    // Specified in microSeconds
		DPRINTF(( "time specified -> %fus\n", width));
        if (*width_arg == '+') {
            double current = (servoWidth[servo] *  (servoMaxPulseUSec - servoMinPulseUSec) + servoMinPulseUSec);
            DPRINTF(( "Add %f to %f = %f\n", width, current, current + width));
            width = current + width;
        } else if (*width_arg == '-') {
            double current = (servoWidth[servo] *  (servoMaxPulseUSec - servoMinPulseUSec) + servoMinPulseUSec);
            DPRINTF(( "Subtract %f from %f = %f\n", width, current, current - width));
            width = current - width;
        }
        // convert to fraction
        width = (width - servoMinPulseUSec) / (servoMaxPulseUSec - servoMinPulseUSec)	;	
	} else if (!strcmp(p, "%")) {
	    // Specified in percentage of total allowed range
	    width = width / 100.0;
        if (*width_arg == '+') {
            double current = servoWidth[servo];
            DPRINTF(( "Add %f to %f = %f\n", width, current, current + width));
            width = current + width;
        } else if (*width_arg == '-') {
            double current = servoWidth[servo];
            DPRINTF(( "Subtract %f from %f = %f\n", width, current, current - width));
            width = current - width;
        }
		DPRINTF(( "%% specified -> %f\n", width));
	} else {
		return -1;
	}

	if (width == 0) {
		return (int)width;
	} else if (width < 0.0 || width > 1.0) {
		return -1;
	} else {
		return width;
	}
}

static void processLoop(void) {
    // This is the main real loop, where we read any incoming data on PCADEVICEFILE
    // and parse it for commands.
	int fd;
	static char line[1024];
	int numChars = 0;

	if ((fd = open(PCADEVICEFILE, O_RDWR|O_NONBLOCK)) == -1)
		fatal("PCA9685servod: Failed to open %s: %m\n", PCADEVICEFILE);

	for (;;) { // endlessly repeat myself endlessly repeating myself...
		int n, servo;
		double width;
		fd_set ifds;
		char width_arg[100];

        // prepare the file descriptors to read any incoming commands
		FD_ZERO(&ifds);
		FD_SET(fd, &ifds);

        // use select to wait on incoming data; skip the rest of the loop if
        // it returns anything other than 1
		if ((n = select(fd+1, &ifds, NULL, NULL, NULL)) != 1)
			continue; 
		// sit on the file descriptor until something is read or the timeout expires - 
		// and since we have set no timeout that could be a long time
		while (read(fd, line+numChars, 1) == 1) {
		    // read one byte at a time until there is a \n
		    // and then process what should be a command
			if (line[numChars] == '\n') {
			    // make sure to terminate the input in the hope it will stop 
			    // buffer over-runs
			    // zero 'nchars' ready for the next time 
				line[++numChars] = '\0';
				numChars = 0;
                n = sscanf(line, "%d=%s", &servo, width_arg);
                if (n != 2) {
                    fprintf(stderr, "Bad input: %s", line);
                } else if (servo < 0 || servo >= MAX_SERVOS) {
                    fprintf(stderr, "Invalid servo number %d\n", servo);
                } else if ((width = parse_width(servo, width_arg)) < 0) {
                    fprintf(stderr, "Invalid width (%f) specified\n", width);
                } else {
                    set_servo(servo, width);
                }
				
			} else {
			    // increment the char count
			    // if it gets too big, chop it back to 0 as a brutal
			    // but effective preventative of buffer overrun
				if (++numChars >= 1022) {
					fprintf(stderr, "Too much input; tossing out first 1022 chars. Be more careful!\n");
					numChars = 0;
				}
			}
		}
	}
}

// parse the user-supplied value for the min or max pulse timing; return a value in uS
static double parsePulseTimingArgs(char *arg) {
	char *p;
	double val = strtod(arg, &p);

	if (*arg < '0' || *arg > '9' || val < 0) {
		return (double)0.0;
	} else if (*p == '\0') {
		return val * stepTimeUSec;
	} else if (!strcmp(p, "us")) {
		return val;
	} else if (!strcmp(p, "%")) {
	    if (val < 0) val = (double)0.0;
	    if (val > 100) val = (double)100.0;
		return (val * cycleTimeUSec / 100.0 );
	} else {
		fatal("Invalid min/max value specified\n");
	}

	return -1; // Never reached, at least shouldn't be!
}

int main(int argc, char **argv) {
	char *servoMinPulseArg = NULL;
	char *servoMaxPulseArg = NULL;
	char *cycleTimeArg = NULL;
	char *stepTimeArg = NULL;
	char *i2c_address_arg = NULL;
	char *p;
	int  noflicker = 1;

	setvbuf(stdout, NULL, _IOLBF, 0);

	while (1) {
		int c;
		int option_index;

		static struct option long_options[] = {
			{ "help",         no_argument,       0, 'h' },
			{ "noflicker",      no_argument,     0, 'n' },
			{ "min",          required_argument, 0, 'm' },
			{ "max",          required_argument, 0, 'x' },
			{ "cycle-time",   required_argument, 0, 'c' },
			{ "step-size",    required_argument, 0, 's' },
			{ "i2c-device-address",      required_argument, 0, 'a' },
            { 0,              0,                 0, 0   }
		};

		c = getopt_long(argc, argv, "amxhcsf", long_options, &option_index);
		if (c == -1) {
			break;
		} else if (c == 'c') {
			cycleTimeArg = optarg;
		} else if (c == 's') {
			stepTimeArg = optarg;
		} else if (c == 'm') {
			servoMinPulseArg = optarg;
		} else if (c == 'x') {
			servoMaxPulseArg = optarg;
		} else if (c == 'a') {
			i2c_address_arg = optarg;
		} else if (c== 'n') {
		    noflicker = 0;
		} else if (c == 'h') {
			printf("\nUsage: %s <options>\n\n"
				"Options:\n"
				"  --help              this incredibly helpful message\n"
				"  --cycle-time=Nus    control pulse cycle time in microseconds, default\n"
				"                      %dus. Max is 41666, min is 655. The hardware may\n"
				"                      not provide exactly your requested value\n"
				"  --step-size=Nus     Pulse width increment step size in microseconds,\n"
				"                      default %dus\n"
				" --i2c-device-address PCA9685 devices can be set to use an i2c address\n"
				"                      other than the default of %0x\n"
				"  --min={N|Nus|N%%}   the minimum allowed pulse width, default %d steps or %dus\n"
				"  --max={N|Nus|N%%}    the maximum allowed pulse width, default %d steps or %dus\n"
				"min and max values can be specified in units of steps, in microseconds,\n"
				"or as a percentage of the cycle time.  So, for example, if cycle time is\n"
				"20000us and step size is 10us then the following are equivalent:\n\n"
				"          --min=50   --min=500us    --min=2.5%%\n\n"
				"For the default configuration, example commands to set the first servo\n"
				"to the mid position would be any of:\n\n"
				"  echo 0=150 >    /dev/pca9685servo     # as a number of steps\n"
				"  echo 0=50%% >   /dev/pca9685servo     # as a percentage\n"
				"  echo 0=1500us > /dev/pca9685servo     # as microseconds\n"
				"Servo position can be set  relative to the current\n"
				"position by adding a '+' or '-' in front of the width:\n"
				"  echo 0=+10%% > /dev/pca9685servo\n"
				"  echo 0=-20 > /dev/pca9685servo\n\n"
				" --noflicker          set all outputs to start their cycle at the same time\n"
				"                      which may reduce flicker when driving a number of LEDS\n\n",
				argv[0],
				DEFAULT_cycleTimeUSec,
				DEFAULT_stepTimeUSec,
				DEFAULT_PCA_ADDR,
				DEFAULT_servoMinPulseUSec/DEFAULT_stepTimeUSec, DEFAULT_servoMinPulseUSec,
				DEFAULT_servoMaxPulseUSec/DEFAULT_stepTimeUSec, DEFAULT_servoMaxPulseUSec);
			exit(0);
		} else {
			fatal("Invalid parameter\n");
		}
	}

        // The PCA9685 can be configured to use several other addresses with hardware
        // pin-fiddling; values are between 0x40 & 0x4F with a couple disallowed
	if (i2c_address_arg) {
		i2c_address = (unsigned int)(strtol(i2c_address_arg, &p, 0));
		// note the 0 base value here that allows users to provide 
		// a hex or other based address value. Learning this was painful.
	} else {
		i2c_address = DEFAULT_PCA_ADDR;
	}

	if (cycleTimeArg) {
		cycleTimeUSec = strtol(cycleTimeArg, &p, 10);
		if (*cycleTimeArg < '0' || *cycleTimeArg > '9' ||
				(*p && strcmp(p, "us")) ||
				cycleTimeUSec < MIN_cycleTimeUSec ||
				cycleTimeUSec > MAX_cycleTimeUSec)
			fatal("Invalid cycle-time specified\n");
	} else {
		cycleTimeUSec = DEFAULT_cycleTimeUSec;
	}

	if (stepTimeArg) {
		stepTimeUSec = strtol(stepTimeArg, &p, 10);
		if (*stepTimeArg < '0' || *stepTimeArg > '9' ||
				(*p && strcmp(p, "us")) ||
				stepTimeUSec < 2 || stepTimeUSec > 1000) {
			fatal("Invalid step-size specified\n");
		}
	} else {
		stepTimeUSec = DEFAULT_stepTimeUSec;
	}
	
	calculateTimerSettings(cycleTimeUSec);

	if (cycleTimeUSec / stepTimeUSec < 100) {
		fatal("PCA cycle time must be at least 100 * step-size\n");
	}

	if (servoMinPulseArg) {
		servoMinPulseUSec = parsePulseTimingArgs(servoMinPulseArg);
	} else {
		servoMinPulseUSec = DEFAULT_servoMinPulseUSec;
	}

	if (servoMaxPulseArg) {
		servoMaxPulseUSec = parsePulseTimingArgs(servoMaxPulseArg);
	} else {
		servoMaxPulseUSec = DEFAULT_servoMaxPulseUSec;
	}

	if (servoMaxPulseUSec > cycleTimeUSec) {
		fatal("max value is larger than cycle time\n");
	}
	if (servoMinPulseUSec >= servoMaxPulseUSec) {
		fatal("min value is >= max value\n");
	}
	if (servoMinPulseUSec < 0) {
		fatal("min value is too small\n");
	}

	fprintf(stderr, "Device address = 0x%02x\n", i2c_address);
	fprintf(stderr, "Requested servo cycle time: %8.3fus\n", cycleTimeUSec);
	fprintf(stderr, "Pulse increment step size: %8.3fus\n", stepTimeUSec);
	fprintf(stderr, "Minimum width value:       %8.3fus (%d)\n", servoMinPulseUSec,
						(int)(servoMinPulseUSec / stepTimeUSec));
	fprintf(stderr, "Maximum width value:       %8.3fus (%d)\n", servoMaxPulseUSec,
						(int)(servoMaxPulseUSec / stepTimeUSec));

	setup_sighandlers();
	
	init_servo_starts(noflicker);
	init_hardware();

	unlink(PCADEVICEFILE);
	if (mkfifo(PCADEVICEFILE, 0666) < 0)
		fatal("pca9685servod: Failed to create %s: %m\n", PCADEVICEFILE);
	if (chmod(PCADEVICEFILE, 0666) < 0)
		fatal("pca9685servod: Failed to set permissions on %s: %m\n", PCADEVICEFILE);

	if (daemon(0,1) < 0)
		fatal("pca9685servod: Failed to daemonize process: %m\n");

	processLoop();

	return 0;
}

