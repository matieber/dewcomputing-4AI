#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>

void ParseData(char * Data_String);

#define NUM_OF_16B_REG   12             // RS232: 12; Trifasico:26; X-5:40;
#define SLAVE_ID         1
#define BAUDRATE         CBR_9600
#define TX_BUFFER_SIZE          2800
#define RX_BUFFER_SIZE          2800
//#define START_CHAR              '!'
//#define END_CHAR                '*'  


//struct timeval tv1, tv2;
double time_diff;

typedef struct{
	unsigned long int VEF, IEF, Pact, Pap, FP, KWh;
} phase_data_t;

/* Table of CRC values for high-order byte */ 
const unsigned char auchCRCHi[] = { 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40}; 
/* Table of CRC values for low-order byte */ 
const unsigned char auchCRCLo[] = { 
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40};

unsigned short int CRC16(char * , unsigned char );

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf(stderr,"error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                fprintf(stderr,"error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

int main (void){
    
    size_t i;
    FILE * Mediciones_txt, * Mediciones_csv;
    char com[]={"/dev/ttyUSB0"};
	
    fprintf(stderr,"Bienvenido a Power Meter Software\n");
    fprintf(stderr,"Abriendo puerto\n");
    sleep(2);
    
	int fd; /* File descriptor for the port */
    
    fd = open(com, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd == -1)
	{
		/* Could not open the port. */
		perror("open_port: Unable to open /dev/ttyUSB0");
	}
	else
		fcntl(fd, F_SETFL, 0);

    set_interface_attribs (fd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)    
    
    char Rx_Buffer[RX_BUFFER_SIZE];
    char Rx_char;
    unsigned int n_rx;
    
    fprintf(stderr,"OK\nEstableciendo comunicacion con el medidor... \n");
    fprintf(stdout,"VEF,IEF,Pact,Pap,FP,KWh\n");
    fflush(stdout);
    i=0;
    unsigned short int CRC_Calc;
    while( 1 ){
        
        char Tx_String[]={SLAVE_ID,3,0,0,0,NUM_OF_16B_REG,0,0};
        CRC_Calc=CRC16(Tx_String,6);
        Tx_String[6]=CRC_Calc&0x00FF;
        Tx_String[7]=(CRC_Calc>>8)&0x00FF;
        for(i=0; i<8; i++){
            //Envio comando
            if(!write(fd, &(Tx_String[i]), 1)){
                fprintf(stderr,"Error al enviar datos por el puerto serie.");
                sleep(5000);
                return 1;
            }//end if
        }//end for
        
        
        //Recepcion
        for(i=0; i<2*NUM_OF_16B_REG+5; i++){      // ... SLAVE_ADDRESS(1) + FUNCTION_CODE(1) + BYTE_COUNT(1) + CRC(2)
            n_rx=1;
            if(!read(fd, &Rx_char, n_rx )){
                fprintf(stderr,"Error al recibir datos por el puerto serie.");
                sleep(5000);
                return 1;  
            }//end if        
            Rx_Buffer[i]=Rx_char;
        }//for
        ParseData(&(Rx_Buffer[3]));
        CRC_Calc=CRC16(Rx_Buffer,2*NUM_OF_16B_REG+3);
        usleep(500 * 1000);
        
    }//end while(1)
    fclose(Mediciones_csv);
    close(fd);
    
}//end main

//------------------------------------------------------------------------------
void ParseData(char * Data_String){
     phase_data_t Phase_R, Phase_S, Phase_T;
     char date, month, year, hours, minutes, seconds;
     char CRC_Hi, CRC_Lo;
     char Slave_ID, RS232_Reg;
          
     Slave_ID=Data_String[0];
     RS232_Reg=Data_String[1];
     date=(Data_String[2]&0x0F)+(Data_String[2]>>4)*10;
     month=(Data_String[3]&0x0F)+(Data_String[3]>>4)*10;
     year=(Data_String[4]&0x0F)+(Data_String[4]>>4)*10;
     hours=(Data_String[6]&0x0F)+((Data_String[6]>>4)&0x01)*10;
     minutes=(Data_String[7]&0x0F)+(Data_String[7]>>4)*10;
     seconds=(Data_String[8]&0x0F)+((Data_String[8]>>4)&0x07)*10;
     
     Phase_R.VEF=Data_String[10]&0xFF;
     Phase_R.IEF=(256*Data_String[13]+Data_String[12])&0xFFFF;
     Phase_R.Pact=(256*Data_String[15]+Data_String[14])&0xFFFF;
     Phase_R.Pap=(256*Data_String[17]+Data_String[16])&0xFFFF;
     Phase_R.FP=Data_String[18]&0xFF;
     Phase_R.KWh=(256*256*Data_String[22]+256*Data_String[21]+Data_String[20])&0xFFFFFF;
     
     Phase_S.VEF=Data_String[24]&0xFF;
     Phase_S.IEF=(256*Data_String[27]+Data_String[26])&0xFFFF;
     Phase_S.Pact=(256*Data_String[29]+Data_String[28])&0xFFFF;
     Phase_S.Pap=(256*Data_String[31]+Data_String[30])&0xFFFF;
     Phase_S.FP=Data_String[32]&0xFF;
     Phase_S.KWh=(256*256*Data_String[36]+256*Data_String[35]+Data_String[34])&0xFFFFFF;
     
     Phase_T.VEF=Data_String[38]&0xFF;
     Phase_T.IEF=(256*Data_String[41]+Data_String[40])&0xFFFF;
     Phase_T.Pact=(256*Data_String[43]+Data_String[42])&0xFFFF;
     Phase_T.Pap=(256*Data_String[45]+Data_String[44])&0xFFFF;
     Phase_T.FP=Data_String[46]&0xFF;
     Phase_T.KWh=(256*256*Data_String[50]+256*Data_String[49]+Data_String[48])&0xFFFFFF;
     
     CRC_Lo=Data_String[52];
     CRC_Hi=Data_String[53];

     fprintf(stdout,"%zu,%zu,%zu,%zu,%zu,%zu\n",Phase_R.VEF,Phase_R.IEF,Phase_R.Pact,Phase_R.Pap,Phase_R.FP,Phase_R.KWh);
     fflush(stdout);

     //printf("Slave ID: %i; Baudrate: %i\n",Slave_ID,(RS232_Reg>>5)&0x07);
     //printf("%i-%i-%i, %i:%i:%i \n",date,month,year,hours,minutes,seconds);
     //printf("R: VEF=%zu, IEF=%zu, Pact=%zu, Pap=%zu, FP=%zu, KWh=%zu \n",Phase_R.VEF,Phase_R.IEF,Phase_R.Pact,Phase_R.Pap,Phase_R.FP,Phase_R.KWh);
    // printf("S: VEF=%i, IEF=%i, Pact=%i, Pap=%i, FP=%i, KWh=%i \n",Phase_S.VEF,Phase_S.IEF,Phase_S.Pact,Phase_S.Pap,Phase_S.FP,Phase_S.KWh);
    // printf("T: VEF=%i, IEF=%i, Pact=%i, Pap=%i, FP=%i, KWh=%i \n",Phase_T.VEF,Phase_T.IEF,Phase_T.Pact,Phase_T.Pap,Phase_T.FP,Phase_T.KWh);
     //printf("CRC Recibido: %02x %02x\n",CRC_Hi&0xFF,CRC_Lo&0xFF);
     
} 
//------------------------------------------------------------------------------
unsigned short CRC16 (char * puchMsg, unsigned char usDataLen ){ 
    unsigned char uchCRCHi = 0xFF ; 
	unsigned char uchCRCLo = 0xFF ; 
	unsigned char uIndex ; 				 
	unsigned char z;

	for (z=0; z<usDataLen; z++){ 
		uIndex = uchCRCLo ^ (*puchMsg) ; 
		puchMsg++;
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ; 
		uchCRCHi = auchCRCLo[uIndex] ; 
	}//end while
	return (uchCRCHi << 8 | uchCRCLo) ; 
}

