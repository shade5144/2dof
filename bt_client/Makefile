CC = /usr/bin/x86_64-linux-gnu-gcc -B/usr/bin/
LDFLAGS = -lbluetooth -lpthread

all: client.c
	${CC} client.c -o clt ${LDFLAGS}

clean:
	rm clt
