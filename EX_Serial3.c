// C Library Headers
#include <stdio.h>
#include <string.h>

//Linux Headers
#include <fcntl.h>  //contains file controls like 0_RDWR
#include <errno.h>  //Error integer and strerror() function 
#include <termios.h> //contains POSIX terminal control definitions
#include <unistd.h> //write(), read(), close()
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


int main() {
   
    //open the serial port. Change device path as needed(currently set to an standard FTDI USB-UART cable)
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    //create new termios struct, we call it 'tty' for convention
    struct termios tty;

    //Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr : %s\n", errno, strerror(errno)); //%s : string, %i : integer, %d : signed decimal integer
        return 1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 100;    // Wait for up to 10s (100 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    unsigned char msg[3][20]= { { 'H',':','1','\r','\n'}, { 'H',':','2','\r','\n'}, { 'H',':','3','\r','\n'}};
    //unsigned char msg[] = { 'H',':','1','\r','\n'}; 
    char read_buf[3][256];
    memset(&read_buf, '\0', sizeof(read_buf));
    for(int i=0; i<3; i++)
    {
        write(serial_port, msg[i], sizeof(msg[i]));
        printf("Sending message : %s\n", msg[i]);
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
        
        if (num_bytes < 0) {
        printf("Error reading: %s\n", strerror(errno));
        return 1;
        }
        printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf[i]);
        sleep(3);
    }
    close(serial_port);
    return 0; // success
}


