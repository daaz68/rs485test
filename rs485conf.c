/********************************************************************************
 * @file    portconfig.c
 * @author  Daniel Bornaz <daniel.bornaz@gmail.com>
 * @date    ${date}
 * @brief   Communication parameters setup utility
 * @vers    0.1
 ********************************************************************************
 * INCLUDES
 ************************************/
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
#include <termios.h>

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
#define TIOCGRS485            0x542E /* Driver-specific ioctls: */
#define TIOCSRS485            0x542F
#define SER_RS485_USE_GPIO    (1 << 5)
#define GPIO_PIN              77
#define FLAG_MASK             0b11111

/************************************
 * GLOBAL VARIABLES
 ************************************/
struct termios tty; /* Terminal information */
char *devc="/dev/ttyO5";
struct serial_rs485 rs485conf;
int fd;

/************************************
 * FUNCTION PROTOTYPES
 ************************************/
int setup_rs485(void);
void test_rs845(void);
int setup_tty(void);

#if 0
/*
 * kernel serial header serial_rs485 structure
 */
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

/******************************************************************
 *
 */
int main(void){

    printf("UART5 RS485 115200 N81 Configuration Utility v0.1\n");

    /* Open your specific device (e.g., /dev/mydevice): */
    fd = open (devc, O_RDWR);
    if (fd < 0) {
        /* Error handling. See errno. */
        fprintf(stderr,"ERR: Error opening %s.\n",devc);
        exit(1);
    }
    printf("   port %s successfully opened.\n",devc);

    /* Set exclusive access to the device */
    if(flock(fd, LOCK_EX | LOCK_NB) < 0) {
        printf("Error locking %s.\n",devc);
        exit(2);
    }
    printf("   exclusive access on port %s secured.\n",devc);

    setup_rs485();
    printf("   RS485 parameters configured (GPIO77).\n");

    /*
     * stty -F /dev/ttyO5 115200 cs8 -cstopb -parenb
     */
	setup_tty();
    printf("   serial port setup for 115200 N81.\n");

    /* Close the device when finished: */
    if (close (fd) < 0) {
        /* Error handling. See errno. */
        fprintf(stderr,"ERR: Error closing file descriptor.\n");
        exit(3);
    }
    exit(0);
}

/******************************************************************
 * Send some text over the serial port
 */
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

/******************************************************************
 * Configure the rs485 parameters, including the GPIO
 * and line control
 * Always returns zero and exits the program on error
 */
int setup_rs485(void){
    struct serial_rs485 rs485test;

    memset(&rs485conf,0x00,sizeof(struct serial_rs485));

    /* Set GPIO pin to 77 in padding array */
    rs485conf.padding[0]=GPIO_PIN;

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

    /* set up serial parameters */
    if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
        /* Error handling. See errno. */
        fprintf(stderr,"ERR: Error calling ioctl write configuration.\n");
        exit(3);
    }

    /* read and compare the settings */
    memset(&rs485test,0x00,sizeof(struct serial_rs485));

    if (ioctl (fd, TIOCGRS485, &rs485test) < 0) {
        /* Error handling. See errno. */
        fprintf(stderr,"ERR: Error calling ioctl.\n");
        exit(4);
    }

    if ( ((rs485conf.flags && FLAG_MASK) != (rs485test.flags && FLAG_MASK)) ||           /* check the flags */
         (rs485conf.padding[0] != rs485test.padding[0])    /* check the gpio port */
       ) {
        fprintf(stderr,"ERR: Ioctl parameters not configured correctly.\n");
        exit(7);
    }

    return 0;
}

/******************************************************************
 * Configures the tty line parameters to 115200 N81 and the
 * most common used parameters
 */
int setup_tty(void){
    struct termios tty;

    /*
     * Read in existing settings, and handle any error
     */
    if(tcgetattr(fd, &tty) != 0) {
        fprintf(stderr,"ERR: Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(5);
    }

    tty.c_cflag &= ~PARENB;  /* Clear parity bit, disabling parity (most common) */
    tty.c_cflag &= ~CSTOPB;  /* Clear stop field, only one stop bit used in communication (most common) */
    tty.c_cflag &= ~CSIZE;   /* Clear all bits that set the data size */
    tty.c_cflag |= CS8;      /* 8 bits per byte (most common) */
    tty.c_cflag &= ~CRTSCTS; /* Disable RTS/CTS hardware flow control (most common) */
    tty.c_cflag |= CREAD | CLOCAL; /* Turn on READ & ignore ctrl lines (CLOCAL = 1) */

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;   /* Disable echo */
    tty.c_lflag &= ~ECHOE;  /* Disable erasure */
    tty.c_lflag &= ~ECHONL; /* Disable new-line echo */
    tty.c_lflag &= ~ISIG;   /* Disable interpretation of INTR, QUIT and SUSP */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* Turn off s/w flow ctrl */
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); /* Disable any special handling of received bytes */

    tty.c_oflag &= ~OPOST; /* Prevent special interpretation of output bytes (e.g. newline chars) */
    tty.c_oflag &= ~ONLCR; /* Prevent conversion of newline to carriage return/line feed */

    tty.c_cc[VTIME] = 10;  /* Wait for up to 1s (10 deciseconds), returning as soon as any data is received. */
    tty.c_cc[VMIN] = 0;

    /*
     * Set in/out baud rate to be 115200
     */
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    /*
     * Save tty settings, also checking for error
     */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr,"ERR: Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(6);
    }

    return 0; // success
}
