CFLAGS += -Wall -I./
LIBS += -lm -lpthread


CC := aarch64-linux-gnu-gcc
STRIP := aarch64-linux-gnu-strip

BIN = miyoo_inputd

SUBDIRS = .

INCS += $(SUBDIRS)
SRCS:=
SRCS += $(foreach n,$(SUBDIRS),$(wildcard $(n)/*.c))
SRCS += ${wildcard *.c}
CFLAGS = -rdynamic -funwind-tables -D_GNU_SOURCE
CLFAGS += -Werror -Wall  -O3 -std=gnu99
CFLAGS += $(patsubst %,-I%,$(INCS))

OBJS=$(patsubst %.c,%.o,$(SRCS))

.PHONY:


all:$(OBJS)
	$(CC) $(LDFLAGS) -O3 -o  $(BIN) $^
	@$(patsubst %gcc,%strip,$(CC)) --strip-unneeded $(BIN)

clean:
	rm -rf $(OBJS) $(BIN)

install:
	@echo done!

%.o:%.c 
	@echo  CC $@
	$(CC) $(CFLAGS)  -c -o $@ $^  
