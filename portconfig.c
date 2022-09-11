#include <linux/serial.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <linux/serial.h>
#include <unistd.h>
#include <termios.h> // Contains POSIX terminal control definitions

/*
 * kernel serial header serial_rs485 structure
 */
#if 0
struct serial_rs485 {
    __u32   flags;          /* RS485 feature flags */
#define SER_RS485_ENABLED           (1 << 0)    /* If enabled */
#define SER_RS485_RTS_ON_SEND       (1 << 1)    /* Logical level for RTS pin when sending */
#define SER_RS485_RTS_AFTER_SEND    (1 << 2)    /* Logical level for RTS pin after sent*/
#define SER_RS485_RTS_BEFORE_SEND   (1 << 3)
#define SER_RS485_USE_GPIO          (1 << 5)
    __u32   delay_rts_before_send;  /* Delay before send (milliseconds) */
    __u32   delay_rts_after_send;   /* Delay after send (milliseconds) */
    __u32   gpio_pin;               /* GPIO Pin Index */
    __u32   padding[4];     /* Memory is cheap, new structs are a royal PITA .. */
};
#endif

/* Terminal information */
struct termios tty;

/* Driver-specific ioctls: */
#define TIOCGRS485            0x542E
#define TIOCSRS485            0x542F
#define SER_RS485_USE_GPIO    (1 << 5)

char *devc="/dev/ttyO5";
struct serial_rs485 rs485conf;
int fd;

void setup_rs485(void);
void setup_tty(void);
void test_rs845(void);

int main(void){
    struct serial_rs485 rs485test;

    /* Open your specific device (e.g., /dev/mydevice): */
    fd = open (devc, O_RDWR);
    if (fd < 0) {
        /* Error handling. See errno. */
        printf("Error opening %s.\n",devc);
        exit(1);
    }

    /* Set exclusive access to the device */
    if(flock(fd, LOCK_EX | LOCK_NB) < 0) {
        printf("Error locking %s.\n",devc);
        exit(2);
    }

    setup_rs485();

    /* set up serial parameters */
    if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
        /* Error handling. See errno. */
        printf("Error calling ioctl write configuration.\n");
        exit(3);
    }

    /* read and compare the settings */
    memset(&rs485conf,0x00,sizeof(struct serial_rs485));

    if (ioctl (fd, TIOCGRS485, &rs485test) < 0) {
        /* Error handling. See errno. */
        printf("Error calling ioctl.\n");
        exit(4);
    }

    /* Close the device when finished: */
    if (close (fd) < 0) {
        /* Error handling. See errno. */
        printf("Error closing file descriptor.\n");
        exit(3);
    }
    exit(0);
}

/* Do something on the serial port... */
void test_rs845(void){
    unsigned int i;
    char buf[80];

    for( i = 0; i < 100; ++i ) {
        snprintf( buf,79, "%d ", i );
        write( fd, buf, strlen(buf));
        //                sleep(1);
    }
    snprintf( buf,79, "\r\n");
    write( fd, buf, strlen(buf));

}

void setup_rs485(void){

    memset(&rs485conf,0x00,sizeof(struct serial_rs485));

    /* Set GPIO pin to 77 in padding array */
    rs485conf.padding[0]=77;

    /* Enable RS485 mode: */
    rs485conf.flags |= SER_RS485_ENABLED | SER_RS485_USE_GPIO;

    /* Set logical level for RTS pin equal to 1 when sending: */
    rs485conf.flags |= SER_RS485_RTS_ON_SEND;

    /* or, set logical level for RTS pin equal to 0 after sending: */
    rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

    /* Set rts delay before send, if needed: */
    rs485conf.delay_rts_before_send = 0;

    /* Set rts delay after send, if needed: */
    rs485conf.delay_rts_after_send = 0;
}

