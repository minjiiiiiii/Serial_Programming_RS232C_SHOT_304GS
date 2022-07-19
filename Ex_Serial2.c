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


int main() {
    int USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 ) {
       //std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        printf("Error : %s\n",strerror(errno));   
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B9600);
    cfsetispeed (&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
       //std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        printf("Error : %s\n",strerror(errno));   
        }

    //write
    unsigned char cmd[] = "H:1\r\n"; 
    int n_written = 0, spot = 0;

    do {
        n_written = write( USB, &cmd[spot], 1 );
        spot += n_written;
    } while (cmd[spot-1] != '\n' && n_written > 0);   
    
    //read
    int n = 0;
    char buf = '\0';
    
    /* Whole response*/
    char response[1024];
    memset(response, '\0', sizeof response);
    
    do {
        n = read( USB, &buf, 1 );
        sprintf( &response[spot], "%c", buf );
        spot += n;
    } while( buf != '\r' && n > 0);
    
    if (n < 0) {
        //std::cout << "Error reading: " << strerror(errno) << std::endl;
        printf("Error : %s",strerror(errno));   
    }
    else if (n == 0) {
        //std::cout << "Read nothing!" << std::endl;
        printf("Read nothing\n");
    }
    else {
        //std::cout << "Response: " << response << std::endl;
        printf("Respose : %s\n",response);
}
    return 0;
}



