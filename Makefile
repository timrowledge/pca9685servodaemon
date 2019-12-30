.PHONY: all 
all:	pca9685servod

pca9685servod:	pca9685servod.c 
	gcc -Wall -pthread -g -O2 -o pca9685servod pca9685servod.c  -lm -lpigpiod_if2

install: all
# copy the servo daemon to /usr/local/bin
	sudo cp pca9685servod /usr/local/bin
	sudo chmod ugo+x /usr/local/bin/pca9685servod
# make sure the pigpio daemon is enabled and started
	sudo systemctl enable pigpiod
	sudo systemctl start pigpiod
# copy the servod service file and enable the daemon
	sudo cp pca9685servo.service /etc/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl enable pca9685servo
	sudo systemctl start pca9685servo

clean:
	rm -f pca9685servod

