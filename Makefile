CC=g++
CFLAGS=-Wall

main: main.o matrix.o BMA020.o SRF02.o

calibrator: calibrator.o matrix.o BMA020.o

clean:
	rm -f main *.o