COMMON=-O2 -I/usr/include/eigen3 -I./include -I../../include -L../../bin -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS = -lGL -lm -lmujoco -lglfw
CC = g++

ROOT = seed_hand

all:
	$(CC) $(COMMON) ./include/tool_kits_def.cpp main_panda_RH8D.cpp $(LIBS) -o ../../bin/$(ROOT)

main_cpp.o:
	$(CC) $(COMMON) -c main_panda_RH8D.cpp tool_kits.h

tool_kits_def.o:
	$(CC) $(COMMON) -c tool_kits_def.cpp

clean:
	rm *.o ../../bin/$(ROOT)
