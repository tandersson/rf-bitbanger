/****************************************************************************
 ** rfbb_cmd.c ***********************************************************
 ****************************************************************************
 *
 * rfbb_cmd - Utility to control NEXA and other RF remote 
 * receivers through a RF bitbanger interface. Some code is borrowed from
 * rfcmd.
 *
 * Copyright (C) 2010 Tord Andersson <tord.andersson@endian.se>
 *
 * License: GPL v. 2
 *
 * Authors:
 *   Tord Andersson <tord.andersson@endian.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <semaphore.h>
#include <getopt.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>


#define PROG_NAME "rfbb_cmd"
#define PROG_VERSION "0.0.2"

#ifndef TRUE
#define TRUE (1==1)
#endif

#ifndef FALSE
#define FALSE !(TRUE)
#endif

#ifndef BOOL
#define BOOL int
#endif

#define RF_MAX_TX_BITS 4000 /* Max TX pulse/space elements in one message */
#define RF_MAX_RX_BITS 4000 /* Max read RX pulse/space elements at one go */

#define DEFAULT_DEVICE "/dev/rfbb"

/* We will borrow a set of LIRC pulse/space definitions */
#define LIRC_MODE2_SPACE     0x00000000
#define LIRC_MODE2_PULSE     0x01000000
#define LIRC_MODE2_TIMEOUT   0x03000000

#define LIRC_VALUE_MASK      0x00FFFFFF
#define LIRC_MODE2_MASK      0xFF000000

#define LIRC_SPACE(val) (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_SPACE)
#define LIRC_PULSE(val) (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_PULSE)
#define LIRC_TIMEOUT(val) (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_TIMEOUT)

#define LIRC_VALUE(val) ((val)&LIRC_VALUE_MASK)
#define LIRC_MODE2(val) ((val)&LIRC_MODE2_MASK)

#define LIRC_IS_SPACE(val) (LIRC_MODE2(val) == LIRC_MODE2_SPACE)
#define LIRC_IS_PULSE(val) (LIRC_MODE2(val) == LIRC_MODE2_PULSE)
#define LIRC_IS_TIMEOUT(val) (LIRC_MODE2(val) == LIRC_MODE2_TIMEOUT) 

/* Protocol defines */
#define NEXA_SHORT_PERIOD 340  /* microseconds */
#define NEXA_LONG_PERIOD  1020 /* microseconds */
#define NEXA_SYNC_PERIOD  (32 * NEXA_SHORT_PERIOD) /* between frames */
#define NEXA_REPEAT 4

#define SARTANO_SHORT_PERIOD 320  /* microseconds */
#define SARTANO_LONG_PERIOD  960 /* microseconds */
#define SARTANO_SYNC_PERIOD  (32 * SARTANO_SHORT_PERIOD) /* between frames */
#define SARTANO_REPEAT 4


typedef enum {MODE_UNKNOWN, MODE_READ, MODE_WRITE} rfMode_t;
typedef enum {IFC_UNKNOWN, IFC_RFBB, IFC_CUL, IFC_TELLSTICK} rfInterface_t;
typedef enum {PROT_UNKNOWN, PROT_RAW, PROT_NEXA, PROT_PROOVE, PROT_NEXA_L, 
    PROT_SARTANO, PROT_WAVEMAN, PROT_IKEA, PROT_ESIC} rfProtocol_t;
                       
/* LIRC pulse/space element */
typedef int32_t lirc_t; 

/* Local function declarations */
int createNexaBitstream(const char * pHouseStr, const char * pChannelStr,
                     const char * pOn_offStr, BOOL waveman, lirc_t * txBitstream, int * repeatCount);

int createSartanoBitstream(const char * pChannelStr, const char * pOn_offStr,
                        lirc_t * txBitstream, int * repeatCount);
int createIkeaBitstream(const char * pSystemStr, const char * pChannelStr,
                     const char * pLevelStr, const char *pDimStyle,
                     lirc_t * txBitstream, int * repeatCount);
int txBitstream2culStr(lirc_t * pTxBitstream, int txItemCount, int repeatCount, char * txStrCul);

static void printUsage(void);
static void printVersion(void);
static void signalTerminate(int signo);

/* Local variables */
BOOL verbose = FALSE; /* -v option */ 
BOOL stopNow = FALSE;   

/* Command line option handling */
static const char *optString = "d:i:p:rwg:c:l:vh?";

static const struct option longOpts[] = {
    { "device", required_argument, NULL, 'd' },
    { "interface", required_argument, NULL, 'i' },
    { "protocol", required_argument, NULL, 'p' },
    { "read", no_argument, NULL, 'r' },
    { "write", no_argument, NULL, 'w' },
    { "group", required_argument, NULL, 'g' },
    { "channel", required_argument, NULL, 'c' },
    { "serialnumber", required_argument, NULL, 's' },
    { "level", required_argument, NULL, 'l' }, 
    { "verbose", no_argument, NULL, 'v' },
    { "help", no_argument, NULL, 'h' },
    { NULL, no_argument, NULL, 0 }
};

int main( int argc, char **argv )
{
    struct termios tio;
    int fd = -1;
    sem_t * portMutex;
    char SEM_NAME[]= "RFCMD_SEM"; /* Semaphore for multiple access ctrl */
    int opt = 0;
    int longIndex = 0;
    rfInterface_t rfInterface = IFC_RFBB;
    char defaultDevice[255] = DEFAULT_DEVICE;
    char * device = defaultDevice; /* -d option */
    rfMode_t mode = MODE_WRITE; /* read/write */
    char * rfProtocolStr = NULL;
    rfProtocol_t rfProtocol = PROT_NEXA; /* protocol */
    const char * groupStr = NULL;   /* house/group/system opåtion */
    const char * channelStr = NULL; /* -c (channel/unit) option */
    const char * levelStr = NULL; /* level 0 - 100 % or on/off */
    lirc_t txBitstream[RF_MAX_TX_BITS];
    lirc_t rxBitstream[RF_MAX_RX_BITS];
    lirc_t rxValue = 0;
    int rxCount = 0;
    int txItemCount = 0;
    int repeatCount = 0;
    int i;
    char asciiCmdStr[RF_MAX_TX_BITS * 6]; /* hex/ASCII repr is longer than bitstream */
    int asciiCmdLength = 0;

    if(argc < 2)
    {
        printUsage();
        exit(1);    
    }

    opt = getopt_long( argc, argv, optString, longOpts, &longIndex );

    /* parse command options */
    while( opt != -1 ) 
    {
    
        switch( opt ) 
        {
            case 'd':
                if (optarg)
                { 
                    device = optarg;
                }
                else
                {
                    fprintf(stderr, "Error. Missing device path.\n");
                    printUsage();
                    exit(1); 
                }                
                break;

            case 'i':
                if (optarg)
                { 
                    if(strcmp( "RFBB", optarg) == 0)
                    {
                        rfInterface = IFC_RFBB;
                    }
                    else if(strcmp( "CUL", optarg) == 0)
                    {
                        rfInterface = IFC_CUL;
                    }
                    else if(strcmp( "TELLSTICK", optarg ) == 0)
                    {
                        rfInterface = IFC_TELLSTICK;
                    }
                    else
                    {
                        rfInterface = IFC_UNKNOWN;
                        fprintf(stderr, "Error. Unknown interface type: %s\n", optarg);
                        printUsage();
                        exit(1);
                    }
                }
                else
                {
                    fprintf(stderr, "Error. Missing interface type.\n");
                    printUsage();
                    exit(1); 
                }                
                break;  

            case 'r':
                mode = MODE_READ;
                break;

            case 'w':
                mode = MODE_WRITE;
                break;

            case 'p':
                if (optarg)
                {   
                    rfProtocolStr = optarg;
                    if(strcmp( "NEXA", rfProtocolStr) == 0)
                    {
                        rfProtocol = PROT_NEXA;
                    }
                    else if(strcmp( "PROOVE", rfProtocolStr) == 0)
                    {
                        rfProtocol = PROT_NEXA;
                    }
                    else if(strcmp( "WAVEMAN", rfProtocolStr) == 0)
                    {
                        rfProtocol = PROT_WAVEMAN;
                    }
                    else if(strcmp( "SARTANO", rfProtocolStr ) == 0)
                    {
                        rfProtocol = PROT_SARTANO;
                    }
                    else if(strcmp( "NEXA_L", rfProtocolStr ) == 0)
                    {
                        rfProtocol = PROT_NEXA_L;
                    }
                    else
                    {
                        rfProtocol = PROT_UNKNOWN;
                        fprintf(stderr, "Error. Unknown protocol: %s\n", rfProtocolStr);
                        printUsage();
                        exit(1);
                    }
                }
                else
                {
                    fprintf(stderr, "Error. Missing protocol\n");
                    printUsage();
                    exit(1); 
                }                
                break;

            case 'g':
                if (optarg)
                {
                    groupStr = optarg;
                }
                else
                {
                    fprintf(stderr, "Error. Missing group/house/system ID\n");
                    printUsage();
                    exit(1); 
                }                
            break;                

            case 'c':
                if (optarg)
                {
                    channelStr = optarg;
                }
                else
                {
                    fprintf(stderr, "Error. Missing channel number\n");
                    printUsage();
                    exit(1); 
                }                
            break;

            case 'l':
                if (optarg)
                {
                    levelStr = optarg;
                }
                else
                {
                    fprintf(stderr, "Error. Missing level\n");
                    printUsage();
                    exit(1); 
                }                
            break;

            case 'v':
                verbose = TRUE;
                break;

            case 'h':   /* Fall through by design */
            case '?':
                printVersion();
                printUsage();
                exit(0);
                break;

            case 0:     /* Long option without a short arg */
                if (longOpts[longIndex].flag != 0)
                  break;
                printf ("option %s", longOpts[longIndex].name);
                if (optarg)
                  printf (" with arg %s", optarg);
                printf ("\n");

            default:
                /* You won't actually get here. */
                break;
        }

        opt = getopt_long( argc, argv, optString, longOpts, &longIndex ); 
    }
      

    /* Build generic transmit bitstream for the selected protocol */
    if(mode == MODE_WRITE)
    {
        switch(rfProtocol)
        {
            case PROT_NEXA:
                if(verbose)
                {
                    printf("NEXA protocol selected\n");
                }
                txItemCount = createNexaBitstream(groupStr, channelStr, levelStr, FALSE, txBitstream, &repeatCount);
                if (txItemCount == 0) {
                    printUsage();
                    exit(1);
                }
                break;

            case PROT_WAVEMAN:
                if(verbose)
                {
                    printf("WAVEMAN protocol selected\n");
                }
                txItemCount = createNexaBitstream(groupStr, channelStr, levelStr, TRUE, txBitstream, &repeatCount);
                if (txItemCount == 0) {
                    printUsage();
                    exit(1);
                }
                break;

            case PROT_SARTANO:
                if(verbose)
                {
                    printf("SARTANO protocol selected\n");
                }
                txItemCount = createSartanoBitstream(channelStr, levelStr, txBitstream, &repeatCount);
                if (txItemCount == 0) {
                    printUsage();
                    exit(1);
                }
                break; 

            case PROT_IKEA:
                if(verbose)
                {
                    printf("IKEA protocol selected\n");
                }
                txItemCount = createIkeaBitstream(groupStr, channelStr, levelStr, "1", txBitstream, &repeatCount);
                if (txItemCount == 0) {
                    printUsage();
                    exit(1);
                }
                break;

            default:
                fprintf(stderr, "Protocol: %s is currently not supported\n", rfProtocolStr);
                printUsage();
                exit(1);             

        }       
    }

        /* create the semaphore - will reuse an existing one if it exists */
        portMutex = sem_open(SEM_NAME,O_CREAT,0644,1);
        if(portMutex == SEM_FAILED)
        {
            fprintf(stderr,  "%s - Error creating port semaphore\n", PROG_NAME);
            perror("Semaphore open error");
            sem_unlink(SEM_NAME);
            exit(1);
        }

        /* lock semaphore to protect port from multiple access */
        if(sem_wait(portMutex) != 0)
        {
                        fprintf(stderr,  "%s - Error aquiring port semaphore\n", PROG_NAME);
                        sem_unlink(SEM_NAME);
                        sem_close(portMutex);
                        exit(1); 
        }

        /* Transmit/read handling for each interface type */ 
        switch (rfInterface) {
          
          case IFC_RFBB:

            if(verbose)
            {
                printf("Selected RFBB interface (RF Bitbanger)\n");
            }

            if( 0 > ( fd = open( device, O_RDWR ) ) ) {
                fprintf(stderr,  "%s - Error opening %s\n", PROG_NAME, device);
                if(sem_post(portMutex) != 0)
                {
                    fprintf(stderr,  "%s - Error releasing port semaphore\n", PROG_NAME);
                }
                sem_unlink(SEM_NAME);
                sem_close(portMutex);
                exit(1);
            }

            if(mode == MODE_WRITE)
            {
                if(verbose)
                {
                    printf("Writing %d pulse_space_items, (%d bytes) to %s\n", txItemCount * repeatCount, txItemCount * 4 * repeatCount, device);
                }
                for(i=0; i<repeatCount; i++)
                {
                    if( write(fd, txBitstream, txItemCount * 4) < 0)
                    {
                         perror("Error writing to RFBB device");
                         break;
                    }               
                }
                sleep(1);               
            }
            else if(mode == MODE_READ)
            {
                stopNow = FALSE;
                if(verbose)
                {
                    printf("Reading pulse_space_items\n");
                }

                /* 
                 * Set up signal handlers to act on CTRL-C events 
                 */
                if (signal(SIGINT, signalTerminate) == SIG_ERR)
                {
                    perror("Can't register signal handler for CTRL-C et al: ");
                    exit (-1);
                } 

                while(stopNow == FALSE) /* repeat until CTRL-C */
                {   
                    rxCount = read(fd, rxBitstream, 4);
                    if(rxCount == 4)
                    {   
                        rxValue = (uint32_t)*&rxBitstream[0];
                        if(LIRC_IS_TIMEOUT(rxValue))
                        {
                            printf("\nRX Timeout");
                        }
                        else if(LIRC_IS_PULSE(rxValue))
                        {
                            printf("\n1 - %05d us", LIRC_VALUE(rxValue));
                        }
                        else if(LIRC_IS_SPACE(rxValue))
                        {
                            printf("\n0 - %05d us", LIRC_VALUE(rxValue));
                        }
                    }
                    else
                    {   
                        if(rxCount == 0)
                        {
                            usleep(100*1000); /* 100 ms */
                            printf(".");
                            fflush(stdout);
                        }
                        else
                        {
                            printf("Read %d bytes\n", rxCount);
                            fflush(stdout);
                        }
                    }
                }               
            }
            printf("\n");
            close(fd);
          break;

          case IFC_TELLSTICK:
            if(verbose)
            {
                printf("Selected Tellstick interface\n");
            }
#if 0
            if( 0 > ( fd = open( *(argv+1), O_RDWR ) ) ) {
                fprintf(stderr,  "%s - Error opening %s\n", PROG_NAME, *(argv+1));
                        if(sem_post(portMutex) != 0)
                        {
                                fprintf(stderr,  "%s - Error releasing port semaphore\n", PROG_NAME);
                }
                            sem_unlink(SEM_NAME);
                            sem_close(portMutex);
                exit(1);
            }

            /* adjust serial port parameters */
            bzero(&tio, sizeof(tio)); /* clear struct for new port settings */
            tio.c_cflag = B4800 | CS8 | CLOCAL | CREAD; /* CREAD not used yet */
            tio.c_iflag = IGNPAR;
            tio.c_oflag = 0;
            tio.c_ispeed = 4800;
            tio.c_ospeed = 4800;
            tcflush(fd, TCIFLUSH);
            tcsetattr(fd,TCSANOW,&tio);
            if( write(fd, txStr, strlen(txStr)) < 0)
            {
                 perror("Error writing to Tellstick device");
            }
            sleep(1); /* one second sleep to avoid device 'choking' */  
            close(fd);
#endif           
          break;
          
          case IFC_CUL:
            if(verbose)
            {
                printf("Selected CUL433 interface\n");
            }

            if( 0 > ( fd = open( device, O_RDWR ) ) ) {
                fprintf(stderr,  "%s - Error opening %s\n", PROG_NAME, device);
                        if(sem_post(portMutex) != 0)
                        {
                                fprintf(stderr,  "%s - Error releasing port semaphore\n", PROG_NAME);
                }
                            sem_unlink(SEM_NAME);
                            sem_close(portMutex);
                exit(1);
            }

            /* adjust serial port parameters */
            bzero(&tio, sizeof(tio)); /* clear struct for new port settings */
            tio.c_cflag = B115200 | CS8 | CLOCAL | CREAD; /* CREAD not used yet */
            tio.c_iflag = IGNPAR;
            tio.c_oflag = 0;
            tio.c_ispeed = 115200;
            tio.c_ospeed = 115200;
            tcflush(fd, TCIFLUSH);
            tcsetattr(fd,TCSANOW,&tio);

            printf("Mode : %d\n", mode);

            if(mode == MODE_WRITE)
            {
     
                /* CUL433 nethome format */
                asciiCmdLength = txBitstream2culStr(txBitstream, txItemCount, repeatCount, asciiCmdStr);

                printf("CUL cmd: %s\n", asciiCmdStr);

                if( write(fd, asciiCmdStr, asciiCmdLength) < 0)
                {
                     perror("Error writing to CUL device");
                }
                sleep(1);
            }
            else if(mode == MODE_READ)
            {
                stopNow = FALSE;
                if(verbose)
                {
                    printf("Reading pulse_space_items\n");
                }

                /*
                 * Set up signal handlers to act on CTRL-C events
                 */
                if (signal(SIGINT, signalTerminate) == SIG_ERR)
                {
                    perror("Can't register signal handler for CTRL-C et al: ");
                    exit (-1);
                }
                /* start rx */
                if( write(fd, "\r\nX01\r\n", 7) < 0)
                {
                     perror("Error issuing RX cmd to CUL device");
                     stopNow = TRUE;
                }

                while(stopNow == FALSE) /* repeat until CTRL-C */
                {
                    rxCount = read(fd, rxBitstream, 5);
                    if(rxCount == 5)
                    {
                        rxValue = (uint32_t)*&rxBitstream[0];
                        printf("\n%08X: ", rxValue);
                        if(rxValue & 0x8000)
                        {
                            printf("1 - %05d us", rxValue & 0x7FFF);
                        }
                        else
                        {
                            printf("0 - %05d us", rxValue & 0x7FFF);
                        }
                        if((rxValue & 0x7FFF) == 0x7FFF)
                        {
                        	printf(" - Timeout");
                        }
                    }
                    else
                    {
                        if(rxCount == 0)
                        {
                            usleep(100*1000); /* 100 ms */
                            printf(".");
                            fflush(stdout);
                        }
                        else
                        {
                            printf("Read %d bytes\n", rxCount);
                            fflush(stdout);
                        }
                    }
                }
            }
            close(fd);
          break;
          
          
          default:
                fprintf(stderr,  "%s - Illegal interface type (%d)\n", PROG_NAME, rfInterface);         
          break;
          
    
        }
        
        
        /* Unlock semaphore */
            if(sem_post(portMutex) != 0)
            {
                fprintf(stderr,  "%s - Error releasing port semaphore\n", PROG_NAME);
                sem_unlink(SEM_NAME);
                sem_close(portMutex);          
                exit(1);
            }
        else
        {
            sem_unlink(SEM_NAME);
            sem_close(portMutex);   
        }

  exit(0);
}


int createNexaBitstream(const char * pHouseStr, const char * pChannelStr,
                     const char * pOn_offStr, BOOL waveman, lirc_t * pTxBitstream, int * repeatCount)
{
    int houseCode;
    int channelCode;
    int on_offCode;
    int txCode = 0;
    const int unknownCode =  0x6;
    int bit;
    int bitmask = 0x0001;
    int itemCount = 0;

    *repeatCount = NEXA_REPEAT;

    houseCode = (int)((* pHouseStr) - 65);  /* House 'A'..'P' */
    channelCode = atoi(pChannelStr) - 1;    /* Channel 1..16 */
    on_offCode =  atoi(pOn_offStr);         /* ON/OFF 0..1 */

    if(verbose)
    {
        printf("House: %d, channel: %d, on_off: %d\n", houseCode, channelCode, on_offCode);
    }

    /* check converted parameters for validity */
    if((houseCode < 0) || (houseCode > 15) ||      // House 'A'..'P'
       (channelCode < 0) || (channelCode > 15) ||
       (on_offCode < 0) || (on_offCode > 1))
    {
        fprintf(stderr,"Invalid group (house), channel or on/off code\n");
        return 0;
    } 
    else 
    {
        /* b0..b11 txCode where 'X' will be represented by 1 for simplicity.
           b0 will be sent first */
        txCode = houseCode;
        txCode |= (channelCode <<  4);
        if (waveman && on_offCode == 0) {
        } else {
            txCode |= (unknownCode <<  8);
            txCode |= (on_offCode  << 11);
        }

        /* convert to send cmd bitstream */
        for(bit=0;bit<12;bit++)
        {
            if((bitmask & txCode) == 0) {
                /* bit timing might need further refinement */
                /* 340 us high, 1020 us low,  340 us high, 1020 us low */
                pTxBitstream[itemCount++] = LIRC_PULSE(NEXA_SHORT_PERIOD);
                pTxBitstream[itemCount++] = LIRC_SPACE(NEXA_LONG_PERIOD); 
                pTxBitstream[itemCount++] = LIRC_PULSE(NEXA_SHORT_PERIOD);
                pTxBitstream[itemCount++] = LIRC_SPACE(NEXA_LONG_PERIOD);             
            } 
            else 
            { /* add 'X' (floating bit) */
                /* 340 us high, 1020 us low, 1020 us high,  350 us low */
                pTxBitstream[itemCount++] = LIRC_PULSE(NEXA_SHORT_PERIOD);
                pTxBitstream[itemCount++] = LIRC_SPACE(NEXA_LONG_PERIOD); 
                pTxBitstream[itemCount++] = LIRC_PULSE(NEXA_LONG_PERIOD);
                pTxBitstream[itemCount++] = LIRC_SPACE(NEXA_SHORT_PERIOD);              
            }
            bitmask = bitmask<<1;
        }
        /* add stop/sync bit and command termination char '+'*/
        pTxBitstream[itemCount++] = LIRC_PULSE(NEXA_SHORT_PERIOD);
        pTxBitstream[itemCount++] = LIRC_SPACE(NEXA_SYNC_PERIOD);
    }

    return itemCount;
}

int createSartanoBitstream(const char * pChannelStr, const char * pOn_offStr,
                        lirc_t * pTxBitstream, int * repeatCount)
{
    int itemCount = 0;
    int on_offCode;
    int bit;
    
    on_offCode =  atoi(pOn_offStr);         /* ON/OFF 0..1 */
    *repeatCount = SARTANO_REPEAT;

    if(verbose)
    {
        printf("Channel: %s, on_off: %d\n", pChannelStr, on_offCode);
    }

    /* check converted parameters for validity */
    if((strlen(pChannelStr) != 10) ||
       (on_offCode < 0) || (on_offCode > 1)) 
    {
        fprintf(stderr,"Invalid channel or on/off code\n");
        return 0;
    } 
    else 
    {
        for(bit=0;bit<=9;bit++)
        {
            /* "1" bit */
            if(strncmp(pChannelStr+bit, "1", 1) == 0) 
            { 
                pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
                pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD); 
                pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
                pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD);  
            }
            /* "0" bit */
            else 
            {
                pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
                pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD); 
                pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_LONG_PERIOD);
                pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_SHORT_PERIOD);
            }
        }
        if (on_offCode >= 1)
        {
            /* ON == "10" */
            pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
            pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD); 
            pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
            pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD);
            pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
            pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD); 
            pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_LONG_PERIOD);
            pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_SHORT_PERIOD);            
        }    
        else
        {
            /* OFF == "01" */
            pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
            pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD); 
            pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_LONG_PERIOD);
            pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_SHORT_PERIOD);  
            pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
            pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD); 
            pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
            pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_LONG_PERIOD);            
        }
            
        /* add stop/sync bit and command termination char '+'*/
        pTxBitstream[itemCount++] = LIRC_PULSE(SARTANO_SHORT_PERIOD);
        pTxBitstream[itemCount++] = LIRC_SPACE(SARTANO_SYNC_PERIOD);
    }
    
    return itemCount;
}

int createIkeaBitstream(const char * pSystemStr, const char * pChannelStr,
                     const char * pLevelStr, const char *pDimStyle,
                     lirc_t * txBitstream, int * repeatCount)
{
#if 0
    *pStrReturn = '\0'; /* Make sure tx Bitstream is empty */

    const char STARTCODE[] = "STTTTTT�";
    const char TT[]  = "TT";
    const char A[]   = "�";
    int systemCode   = atoi(pSystemStr) - 1;   /* System 1..16 */
    int channelCode  = atoi(pChannelStr);  /* Channel 1..10 */
    int Level        = atoi(pLevelStr);     /* off,10,20,..,90,on */
    int DimStyle     = atoi(pDimStyle);
    int intCode      = 0;
    int checksum1    = 0;
    int checksum2    = 0;
    int intFade ;
    int i ;
    int rawChannelCode = 0;

    /* check converted parameters for validity */
    if ( (channelCode <= 0) || (channelCode > 10) ||
         (systemCode < 0) || (systemCode > 15) ||
         (Level < 0) || (Level > 10) ||
         (DimStyle < 0) || (DimStyle > 1))
    {
        return 0;
    }

    if (channelCode == 10) {
        channelCode = 0;
    }
    rawChannelCode = (1<<(9-channelCode));

    strcat(pStrReturn, STARTCODE ) ; //Startcode, always like this;
    intCode = (systemCode << 10) | rawChannelCode;

    for ( i = 13; i >= 0; --i) {
        if ((intCode>>i) & 1) {
            strcat(pStrReturn, TT );
            if (i % 2 == 0) {
                checksum2++;
            } else {
                checksum1++;
            }
        } else {
            strcat(pStrReturn,A);
        }
    }

    if (checksum1 %2 == 0) {
        strcat(pStrReturn, TT );
    } else {
        strcat(pStrReturn, A) ; //1st checksum
    }

    if (checksum2 %2 == 0) {
        strcat(pStrReturn, TT );
    } else {
        strcat(pStrReturn, A ) ; //2nd checksum
    }

    if (DimStyle == 1) {
        intFade = 11 << 4; //Smooth
    } else {
        intFade = 1 << 4; //Instant
    }

    switch ( Level )
    {
        case 0 :
            intCode = (10 | intFade) ; //Concat level and fade
            break;
        case 1 :
            intCode = (1 | intFade) ; //Concat level and fade
            break;
        case 2 :
            intCode = (2 | intFade) ; //Concat level and fade
            break;
        case 3 :
            intCode = (3 | intFade) ; //Concat level and fade
            break;
        case 4 :
            intCode = (4 | intFade) ; //Concat level and fade
            break;
        case 5 :
            intCode = (5 | intFade) ; //Concat level and fade
            break;
        case 6 :
            intCode = (6 | intFade) ; //Concat level and fade
            break;
        case 7 :
            intCode = (7 | intFade) ; //Concat level and fade
            break;
        case 8 :
            intCode = (8 | intFade) ; //Concat level and fade
            break;
        case 9 :
            intCode = (9 | intFade) ; //Concat level and fade
            break;
        case 10 :
        default :
            intCode = (0 | intFade) ; //Concat level and fade
            break;
    }

    checksum1 = 0;
    checksum2 = 0;

    for (i = 0; i < 6; ++i) {
        if ((intCode>>i) & 1) {
            strcat(pStrReturn, TT);

            if (i % 2 == 0) {
                checksum1++;
            } else {
                checksum2++;
            }
        } else {
            strcat(pStrReturn, A );
        }
    }

    if (checksum1 %2 == 0) {
        strcat(pStrReturn, TT);
    } else {
        strcat(pStrReturn, A ) ; //2nd checksum
    }

    if (checksum2 %2 == 0) {
        strcat(pStrReturn, TT );
    } else {
        strcat(pStrReturn, A ) ; //2nd checksum
    }

    strcat(pStrReturn, "+");

    return strlen(pStrReturn); 
#endif
    return 0;
}

/* Convert generic bitstream format to CUL433 format */ 
int txBitstream2culStr(lirc_t * pTxBitstream, int txItemCount, int repeatCount, char * txStrCul)
{
    int i;
    int pulses = 0;
    char * pCulStr = txStrCul;
    char tmpStr [20];
    
    * pCulStr = '\0';

    strcat(pCulStr, "\r\nX01\r\n"); /* start radio */
    strcat(pCulStr, "E\r\n"); /* empty tx buffer */ 

    for(i=0;i<txItemCount;i++)
    {
        sprintf(tmpStr, "%04X", LIRC_VALUE(pTxBitstream[i]));

        if ( LIRC_IS_PULSE ( pTxBitstream[i] ) == TRUE )
        {
            strcat ( pCulStr, "A" );
            strcat ( pCulStr, tmpStr );
            pulses++;
        }
        else /* low */
        {
            strcat ( pCulStr, tmpStr );
            strcat ( pCulStr, "\r\n" );
        }                

    }
    
    if(pulses > 1)
    {
      
        /* number of repetitions */
        sprintf(tmpStr, "S%02d\r\n", repeatCount);
        strcat(pCulStr, tmpStr);	
        return strlen(pCulStr);
    }
    else
    {
        txStrCul[0] = '\0';
        return 0;
    }
}

static void printUsage(void)
{
    printf("\nUsage: %s <-diprwgcslvh> [value]\n", PROG_NAME);
    printf("\t -d --device <path> defaults to %s\n", DEFAULT_DEVICE);
    printf("\t -i --interface. RFBB, CUL or TELLSTICK. Defaults to RFBB (RF Bitbanger)\n");
    printf("\t -p --protocol. NEXA, NEXA_L, SARTANO, WAVEMAN, IKEA or RAW\n"); 
    printf("\t -r --read. Raw space/pulse reading. For interfaces above that supports reading\n");
    printf("\t -w --write. Send command (default)\n");  
    printf("\t -g --group. The group/house/system number or letter\n");
    printf("\t -c --channel. The channel/unit number\n");
    printf("\t -s --serialnumber. The serial/unique number used by NEXA L (self-learning)\n");
    printf("\t -l --level. 0 - 100. All values above 0 will switch on non dimmable devices\n");
    printf("\t -v --verbose\n");
    printf("\t -h --help\n");
    printf("\n\t Some useful protocol arguments - NEXA, WAVEMAN:\n");
    printf("\t\tgroup: A..P\n\t\tchannel: 1..16\n\t\toff/on: 0..1\n" );
    printf("\n");
    printf("\t Protocol arguments - SARTANO:\n");
    printf("\t\tchannel: 0000000000..1111111111\n\t\toff/on: 0..1\n" );
    printf("\n");
    printf("\t Protocol arguments - IKEA:\n");
    printf("\t\tgroup (system): 1..16\n\t\tchannel(device): 1..10\n");
    printf("\t\tlevel: 0..100\n\t\t(dimstyle 0..1)\n\n");
    printf("\tA typical example (NEXA D1 on): %s -d /dev/rfbb -i RFBB -p NEXA -g D -c 1 -l 1\n\n", PROG_NAME);


}

static void printVersion(void) {
    printf("%s (RF Bitbanger cmd tool) v%s\n", PROG_NAME, PROG_VERSION);
    printf("\n");
    printf("Copyright (C) Tord Andersson 2010\n");
    printf("License: GPL v. 2\n");
    printf("Written by Tord Andersson. Code fragments from rfcmd by Tord Andersson, Micke Prag, \nGudmund Berggren, Tapani Rintala and others\n");
}

static void signalTerminate(int signo)
{
    /*
    * This will force the exit handler to run
    */   
    if(verbose)
    {
        printf("Signal handler for %d signal\n", signo);
    }
    stopNow = TRUE;
}

