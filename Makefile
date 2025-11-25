CC = gcc
CFLAGS = -Wall -Wextra -O2 -I/usr/include/modbus
LDFLAGS = -lmodbus

THERMIA_ADDRESS=192.168.0.106
THERMIA_TYPE=mega

thermia: common_thermia.c common_thermia.h common_thermia_registers.h
	$(CC) $(CFLAGS) -DTEST -o thermia common_thermia.c $(LDFLAGS)

test: thermia
	@./thermia $(THERMIA_ADDRESS) $(THERMIA_TYPE) read "valueHeatpumpSoftwareVersionMajor" "valueHeatpumpSoftwareVersionMinor" "valueHeatpumpSoftwareVersionMicro"

format:
	clang-format -i *.[ch]
