TARGET ?= gcc_ntoaarch64le
CFLAGS ?= -Wall -Wextra -O2
OBJS    = main.o supervisor.o ecu_service.o rt_util.o

ecu_sup: $(OBJS)
	qcc -V$(TARGET) $(CFLAGS) -o $@ $(OBJS)

%.o: %.c ecu_sup.h
	qcc -V$(TARGET) $(CFLAGS) -c -o $@ $<

clean:
	rm -f ecu_sup $(OBJS)

.PHONY: clean
