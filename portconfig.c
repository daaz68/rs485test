#include <linux/serial.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <linux/serial.h>
#include <unistd.h>

/* Terminal information */
struct termios tty;

/* Driver-specific ioctls: */
#define TIOCGRS485      0x542E
#define TIOCSRS485      0x542F
#define SER_RS485_USE_GPIO    (1 << 5)

char *devc="/dev/ttyO5";

int main(void){
        unsigned int i;
        char buf[80];
	struct serial_rs485 rs485conf;

	/* Open your specific device (e.g., /dev/mydevice): */
	int fd = open (devc, O_RDWR);
	if (fd < 0) {
		/* Error handling. See errno. */
		printf("Error opening %s.\n",devc);
		exit(1);
	}

	/* Set GPIO pin to 77 */
	rs485conf.padding[0]=77; /* gpio_pin in padding array */

	/* Enable RS485 mode: */
	rs485conf.flags |= SER_RS485_ENABLED | SER_RS485_USE_GPIO;

	/* Set logical level for RTS pin equal to 1 when sending: */
	rs485conf.flags |= SER_RS485_RTS_ON_SEND;

	/* or, set logical level for RTS pin equal to 0 when sending: */
	// rs485conf.flags &= ~(SER_RS485_RTS_ON_SEND);

	/* Set logical level for RTS pin equal to 1 after sending: */
	// rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;

	/* or, set logical level for RTS pin equal to 0 after sending: */
	rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

	/* Set rts delay before send, if needed: */
	rs485conf.delay_rts_before_send = 0;

	/* Set rts delay after send, if needed: */
	rs485conf.delay_rts_after_send = 0;

	/* Set this flag if you want to receive data even whilst sending data */
	rs485conf.flags &= ~(SER_RS485_RX_DURING_TX);

	if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
		/* Error handling. See errno. */
		printf("Error calling ioctl.\n");
		exit(2);
	}

        /* Do something on the serial port... */
        for( i = 0; i < 100; ++i ) {
                snprintf( buf,79, "%d ", i );
                write( fd, buf, strlen(buf));
//                sleep(1);
        }
	snprintf( buf,79, "\r\n");
	write( fd, buf, strlen(buf));

	/* Close the device when finished: */
	if (close (fd) < 0) {
		/* Error handling. See errno. */
		printf("Error closing file descriptor.\n");
		exit(3);
	}
	exit(0);
}
