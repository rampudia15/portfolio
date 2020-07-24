CC=		g++

RM=		rm -f

INCLUDES=	-I.

CFLAGS=		`pkg-config --cflags gstreamer-1.0 opencv`
LDFLAGS=	`pkg-config --libs gstreamer-1.0 opencv`

NAME=		system_demo

SRCS=		src/main.c \
		src/gst_capture.c \
		src/image_processing.c \
		src/controller.c \
		src/motor.c

OBJS=		$(SRCS:.c=.o)

all:		$(MKDIR_P) $(NAME)

$(NAME):	$(OBJS)
		$(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o:		%.c
		$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<
clean:
		$(RM) $(OBJS) $(NAME)

re:		clean all

.PHONY:		all clean re
