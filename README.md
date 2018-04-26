# pca9685servodaemon
A simple daemon for Raspbian to drive a pca9685 pwm card such as those from Sparkfun and Adafruit

Originally inspired by Richad Hirst's servoblaster

This daemon creates /dev/pca9685servo and waits for input. Writing 0=50% to /dev/pca9685 would set the pwm duty cycle of output 0 to 50%.
Read the help info by running the daemon locally with '-h' to see -
  --help              this incredibly helpful message
  --cycle-time=Nus    control pulse cycle time in microseconds, default
                      20,000us. Max is 41666, min is 655. The hardware may
                      not provide exactly your requested value
  --step-size=Nus     Pulse width increment step size in microseconds,
                      default 5us
 --i2c-device-address PCA9685 devices can be set to use an i2c address
                      other than the default of 0x40
  --min={N|Nus|N%%}   the minimum allowed pulse width, default 100 steps or 655us
  --max={N|Nus|N%%}    the maximum allowed pulse width, default 500 steps or 41666dus
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
  
Install and start up with
A) copy the servo daemon to /usr/local/bin
sudo cp pca9685servod /usr/local/bin
sudo chmod ugo+x /usr/local/bin/pca9685servod

B) make sure the pigpio daemon is enabled and started
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

C) copy the servod service file and enable the daemon
sudo cp pca9685servo.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable pca9685servo
sudo systemctl start pca9685servo
