
all: t4main

t4main : t4main.cpp
	cc t4main.cpp -o t4main -lps2pi -lwiringPi
	
