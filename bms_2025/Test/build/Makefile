CC = gcc -g -Wconversion
INC = -I../include -I../../Core/Inc

all: test

test: PackCalculations.o TestAlgorithms.o Fault.o BMSConfig.o
	cc PackCalculations.o TestAlgorithms.o Fault.o BMSConfig.o -lm -o test

PackCalculations.o: ../../Core/Src/PackCalculations.c
	$(CC)  -c ../../Core/Src/PackCalculations.c  $(INC) -o PackCalculations.o

TestAlgorithms.o: ../TestAlgorithms.c
	$(CC) -c ../TestAlgorithms.c $(INC) -o TestAlgorithms.o

BMSConfig.o: ../../Core/Src/BMSconfig.c
	$(CC) -c ../../Core/Src/BMSconfig.c $(INC) -o BMSConfig.o

Fault.o: ../../Core/Src/BMSconfig.c
	$(CC) -c ../../Core/Src/Fault.c $(INC) -o Fault.o

.PHONY : clean
clean:
	rm PackCalculations.o TestAlgorithms.o Fault.o BMSConfig.o

