#rfbb_cmd makefile

EXEC_NAME     = rfbb_cmd
SRCS          = rfbb_cmd.c
CROSS_COMPILE = 
CC            = $(CROSS_COMPILE)gcc
CFLAGS        = -O2 -Wall
LDFLAGS       = 
LIBS          =	-lpthread -lrt
#LIBS         = -lrt
OBJS          = $(SRCS:.c=.o)
TARGET_ROOT   =
INSTALL_DIR   = $(TARGET_ROOT)/usr/local/bin

all: $(EXEC_NAME)

OBJS: $(SRCS:.c=.o)
	$(CC) $(CFLAGS) -c $<

$(EXEC_NAME): $(OBJS)
	$(CC) -o $(EXEC_NAME) ${OBJS} $(LDFLAGS) $(LIBS)


# Install will require root privilegies or sudo
install: $(EXEC_NAME)
	cp $(EXEC_NAME) $(INSTALL_DIR) 	

clean:
	rm -f *.o $(EXEC_NAME) core

distclean:
	rm -f *~
	rm -f *.o $(EXEC_NAME) core

