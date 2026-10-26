# Host unit tests for the platform independent parts of webctrl.
#   make -C components/webctrl/host_test
CC      ?= gcc
CFLAGS  ?= -std=c11 -O1 -g -Wall -Wextra -Werror -Wconversion -Wno-sign-conversion
# make SANITIZE=1 adds ASan/UBSan (needs libasan and libubsan installed)
ifeq ($(SANITIZE),1)
CFLAGS  += -fsanitize=address,undefined -fno-omit-frame-pointer
endif
BUILD   := build

all: run

$(BUILD)/test_webctrl_core: test_webctrl_core.c ../webctrl_core.c ../include/webctrl_core.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I../include test_webctrl_core.c ../webctrl_core.c -lm -o $@

$(BUILD)/test_dns: test_dns.c ../dns_server.c ../dns_server.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DWEBCTRL_HOST_TEST -I.. test_dns.c ../dns_server.c -o $@

run: $(BUILD)/test_webctrl_core $(BUILD)/test_dns
	./$(BUILD)/test_webctrl_core
	./$(BUILD)/test_dns

clean:
	rm -rf $(BUILD)

.PHONY: all run clean
