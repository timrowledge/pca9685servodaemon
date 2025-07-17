# pca9685servodaemon
A simple daemon for Raspbian to drive a pca9685 based pwm card such as those from Sparkfun and Adafruit

Originally inspired by Richad Hirst's servoblaster

This daemon creates /dev/pca9685servo and waits for input. Writing "0=50%" to /dev/pca9685 would set the pwm duty cycle of output 0 to 50%.
(2025-07-17 A warning - the gpio library situation changed with the Pi 5 and it is possible none of this still works.Maybe I'll get around to finxing it)

Clone this repository or download the zip to a suitable place on your Pi. In a terminal run

	make

to simply build the daemon. You can run it manually to test it if you wish. To build and install the daemon run

	make install


This will copy the daemon to /usr/local/bin, enable the pigpiod daemon that we depend on, enable this daemon and start both of them. Once the system is runnng you can test your servo (connected to socket 0 in this case) with a simple

	echo 0=65% > /dev/pca9685servo

Although some modern servos can accept a wider range of motion than is normally specified in the model radio control world, I do urge you to test things out with the default pulse width first. The default min & max are set to 1000uSec and 2000uSec respectively. You may find your servos able to go as far as 500 and 2500uSec.

You can read the help info by running the daemon locally with '-h' to see -

	--help              this incredibly helpful message
	--cycle-time=Nus    control pulse cycle time in microseconds, default
                      20,000us. Max is 41666, min is 655. The hardware may
                      not provide exactly your requested value
	  --step-size=Nus     Pulse width increment step size in microseconds,
                      default 5us
	 --i2c-device-address PCA9685 devices can be set to use an i2c address
                      other than the default of 0x40
	  --min={N|Nus|N%}   the minimum allowed pulse width, default 100 steps or 655us
	  --max={N|Nus|N%}    the maximum allowed pulse width, default 500 steps or 41666dus
	  min and max values can be specified in units of steps, in microseconds,
	  or as a percentage of the cycle time.  So, for example, if cycle time is
	  20000us and step size is 10us then the following are equivalent:
			          --min=50   --min=500us    --min=2.5%

	For the default configuration, example commands to set the first servo
	to the mid position would be any of:
	echo 0=150 >    /dev/pca9685servo    # as a number of steps
	echo 0=50% >   /dev/pca9685servo     # as a percentage
	echo 0=1500us > /dev/pca9685servo    # as microseconds
	
	Servo position can be set  relative to the current
	position by adding a '+' or '-' in front of the width:
	echo 0=+10%% > /dev/pca9685servo
	echo 0=-20 > /dev/pca9685servo	



