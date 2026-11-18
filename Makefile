CC = gcc
CFLAGS = -Wall -Wextra -O2 -I/usr/include/modbus
LDFLAGS = -lmodbus

THERMIA_ADDRESS=192.168.0.106
THERMIA_TYPE=mega

all: thermia thermia-status thermia-caleon

thermia: common_thermia.c common_thermia.h common_thermia_registers.h
	$(CC) $(CFLAGS) -DTEST -o thermia common_thermia.c $(LDFLAGS)

thermia-status: thermia-status.c common_thermia.c common_thermia.h common_thermia_registers.h
	$(CC) $(CFLAGS) -o thermia-status thermia-status.c common_thermia.c $(LDFLAGS)

# the Thermia's comfort wheel from the Caleon (on the Caleon's host, alongside caleon2mqtt): libmodbus-dev, libmosquitto-dev, libcjson-dev
thermia-caleon: thermia-caleon.c common_thermia.c common_thermia.h common_thermia_registers.h
	$(CC) $(CFLAGS) -o thermia-caleon thermia-caleon.c common_thermia.c $(LDFLAGS) -lmosquitto -lcjson -lm

# then: systemctl enable --now thermia-caleon (settings: /etc/default/thermia-caleon, installed once, kept after)
install-caleon: thermia-caleon
	install -m 755 thermia-caleon /usr/local/bin/
	install -m 644 service/thermia-caleon.service /etc/systemd/system/
	[ -f /etc/default/thermia-caleon ] || install -m 644 service/thermia-caleon.default /etc/default/thermia-caleon
	systemctl daemon-reload

test: thermia
	@./thermia $(THERMIA_ADDRESS) $(THERMIA_TYPE) read "valueHeatpumpSoftwareVersionMajor" "valueHeatpumpSoftwareVersionMinor" "valueHeatpumpSoftwareVersionMicro"

format:
	clang-format-19 -i *.[ch]
