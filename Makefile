CC=g++
CFLAGS=-c -Wall
LDFLAGS=

SOURCES_RAPTOR=main.cc matrix.cc BMA020.cc SRF02.cc IMU.cc
OBJECTS_RAPTOR=$(SOURCES_RAPTOR:.cc=.o)

SOURCES_CALIBRATOR=calibrator.cc matrix.cc BMA020.cc
OBJECTS_CALIBRATOR=$(SOURCES_CALIBRATOR:.cc=.o)
	
raptor: $(OBJECTS_RAPTOR)
	$(CC) $(LDFLAGS) $(OBJECTS_RAPTOR) -o raptor
	
calibrator: $(OBJECTS_CALIBRATOR)
	$(CC) $(LDFLAGS) $(OBJECTS_CALIBRATOR) -o calibrator
	
.cc.o:
	$(CC) $(CFLAGS) $< -o $@
	
clean:
	rm -f main *.o