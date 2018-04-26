
.PHONY: all 
all:	pca9685servod

pca9685servod:	pca9685servod.c 
	gcc -Wall -pthread -g -O2 -o pca9685servod pca9685servod.c  -lm -lpigpiod_if2

clean:
	rm -f pca9685servod

