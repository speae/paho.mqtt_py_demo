#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>



#include <sys/time.h>
#include <sys/types.h>
#include <pthread.h>


int main_menu(void);


// for yolo
#include <sys/stat.h>
#define  FIFO_FROM_YOLO   "/tmp/from_yolo_fifo"
#define  FIFO_TO_YOLO     "/tmp/to_yolo_fifo"
#define  BUFF_SIZE   1024

int   counter = 0;
int   fd_from_yolo;
int   fd_to_yolo;
char  buff[BUFF_SIZE];

int     handle;
struct  termios  oldtio,newtio;
    
char    *TitleMessage = "Welcome Serial Port\r\n";
char    Buff[256];
int     RxCount;
int     loop;
int     ending;    
int     key;


// 쓰레드 함수
void *t_function(void *data)
{
    int id;
    int i = 0;
    id = *((int *)data);

    //from yolo
    while(1)
    {
        while(read(fd_from_yolo, buff, BUFF_SIZE) != 0)
        {
            // if(buff[0] == 'a')
            //     Buff[0] = 'f';
            // else if(buff[0] == 'b')
            //     Buff[0] = 'g';
            // else if(buff[0] == 'c')
            //     Buff[0] = 'h';
            // else if(buff[0] == 'd')
            //     Buff[0] = 'i';
            // else if(buff[0] == 'i')
            //     Buff[0] = 'j';
            // else
            Buff[0] = buff[0];
            write( handle, Buff, 1 );
            printf("%s", Buff);
            memset(buff, 0x00, BUFF_SIZE);
        }
    }
}


//------------------------------------------------------------------------
// 설 명 : 어플리케이션 처리 
// 매 계 : 명령행 매계변수
// 반 환 : 없음
// 주 의 : 
//------------------------------------------------------------------------
int main( int argc, char **argv )
{

    struct  termios  oldtio,newtio;
    
    char    *TitleMessage = "Welcome Serial Port\r\n";
    char    Buff[256];
    int     RxCount;
    int     loop;
    int     bluetooth_flag;
    
    int key;
    
    pthread_t p_thread[2];
    int thr_id;
    int status;
    int a = 1;
    int b = 2;

    
    // 쓰레드 생성 아규먼트로 1 을 넘긴다.  
    thr_id = pthread_create(&p_thread[0], NULL, t_function, (void *)&a);
    if (thr_id < 0)
    {
        perror("thread create error : ");
        exit(0);
    }
    //for yolo 
    if ( -1 == ( fd_from_yolo = open( FIFO_FROM_YOLO, O_RDWR)))
    {
        if ( -1 == mkfifo( FIFO_FROM_YOLO, 0666))
        {
            perror( "mkfifo() run error");
            exit( 1);
        }

        if ( -1 == ( fd_from_yolo = open( FIFO_FROM_YOLO, O_RDWR)))
        {
            perror( "open() error");
            exit( 1);
        }
    }
    if ( -1 == ( fd_to_yolo = open( FIFO_TO_YOLO, O_RDWR)))
    {
        if ( -1 == mkfifo( FIFO_TO_YOLO, 0666))
        {
            perror( "mkfifo() run error");
            exit( 1);
        }

        if ( -1 == ( fd_to_yolo = open( FIFO_TO_YOLO, O_RDWR)))
        {
            perror( "open() error");
            exit( 1);
        }
    }


    // 화일을 연다.
    //handle = open( "/dev/ttyUSB1", O_RDWR | O_NOCTTY );
    handle = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );
    //handle = open( "/dev/ttyTHS2", O_RDWR | O_NOCTTY );
    if( handle < 0 ) 
    {
        //화일 열기 실패
        //printf( "Serial Open Fail [/dev/ttyUSB1]\r\n "  );
        printf( "Serial Open Fail [/dev/ttyUSB0]\r\n "  );
        //printf( "Serial Open Fail [/dev/ttyTHS2]\r\n "  );
        exit(0);
    }
    
    tcgetattr( handle, &oldtio );  // 현재 설정을 oldtio에 저장

    memset( &newtio, 0, sizeof(newtio) );
    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD ; 
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    //set input mode (non-canonical, no echo,.....)
    newtio.c_lflag = 0;
    
    newtio.c_cc[VTIME] = 128;    // time-out 값으로 사용된다. time-out 값은 TIME*0.1초 이다.
    newtio.c_cc[VMIN]  = 0;     // MIN은 read가 리턴되기 위한 최소한의 문자 개수
    
    tcflush( handle, TCIFLUSH );
    tcsetattr( handle, TCSANOW, &newtio );
    
    // 타이틀 메세지를 표출한다. 
    write( handle, TitleMessage, strlen( TitleMessage ) );
    
    // 1 문자씩 받아서 되돌린다. 
    bluetooth_flag = 0;
    
    while((key=main_menu()) != 0)
    {
        if (key == 27)
        {
            bluetooth_flag += 1;
        }
        else if (key == 91)
        {
            bluetooth_flag += 1;
        }
        
        switch(key)
        {
        
            case 'i':
               printf("i\n");               
               //Buff[0] = 'i';
               Buff[0] = 'i';
               write( handle, Buff, 1 );
               printf("%c send\n", Buff[0]);
               break; 

            case 'j':
            case 10:
               printf("j\n");               
               //Buff[0] = 'i';
               Buff[0] = 'j';
               write( handle, Buff, 1 );
               printf("%c send\n", Buff[0]);
               break;  

            case 'k':
               printf("k\n");               
               //Buff[0] = 'i';
               Buff[0] = 'k';
               write( handle, Buff, 1 );
               printf("%c send\n", Buff[0]);
               break; 

            case 'A':
            case 97:
                if (bluetooth_flag == 2)
                {
                    printf("forward.\n");
                    Buff[0] = 'i';
                    write( handle, Buff, 1 );
                    printf("%c send\n", Buff[0]);
                    bluetooth_flag = 0;
                    break; 
                }
                else
                {
                    printf("A\n");
                    Buff[0] = 'A';
                    write( handle, Buff, 1 );
                    printf("%c send\n", Buff[0]);
                    bluetooth_flag = 0;
                    break;
                }

	        case 'B':
            case 98:
                if (bluetooth_flag == 2)
                {
                    printf("backward.\n");
                    Buff[0] = 'k';
                    write( handle, Buff, 1 );
                    printf("%c send\n", Buff[0]);
                    bluetooth_flag = 0;
                    break; 
                }
                else
                {
                    printf("B\n");
                    Buff[0] = 'B';
                    write( handle, Buff, 1 );
                    printf("%c send\n", Buff[0]);
                    bluetooth_flag = 0;
                    break;
                }
                
            case 'C':
            case 99:
                if (bluetooth_flag == 2)
                {
                    printf("turn right:: ▶\n");
                    Buff[0] = 'D';
                    write( handle, Buff, 1 );
                    printf("%c send\n", Buff[0]);
                    bluetooth_flag = 0;
                    break; 
                }
                else
                {
                    printf("C\n");               
                    //Buff[0] = 'I';
                    Buff[0] = 'C';
                    write( handle, Buff, 1 );
                    printf("%c send\n", Buff[0]);
                    bluetooth_flag = 0;
                    break;  
                }
                
            case 'D':
            case 100:
                if (bluetooth_flag == 2)
                {
                    printf("◀ ::turn left\n");
                    Buff[0] = 'E';
                    write( handle, Buff, 1 );
                    printf("%c send\n", Buff[0]);
                    bluetooth_flag = 0;
                    break; 
                }
                else
                {
                    printf("turn right:: ▶\n");
                    Buff[0] = 'D';
                    write( handle, Buff, 1 );
                    printf("%c send\n", Buff[0]);
                    bluetooth_flag = 0;
                    break;
                }
                
            case 'E':
            case 101:
               printf("E\n");
               Buff[0] = 'E';
               write( handle, Buff, 1 );
               printf("%c send\n", Buff[0]);
               break;

	        case 'F':
            case 102:
               printf("F\n");
               Buff[0] = 'F';
               write( handle, Buff, 1 );
               printf("%c send\n", Buff[0]);
               break;

            case 'G':
            case 103:
               printf("G\n");               
               //Buff[0] = 'I';
               Buff[0] = 'G';
               write( handle, Buff, 1 );
               printf("%c send\n", Buff[0]);
               break;  

            case 'H':
            case 104:
               printf("H\n");
               //Buff[0] = 'D';
               Buff[0] = 'H';
               write( handle, Buff, 1 );
               printf("%c send\n", Buff[0]);
               break;

            case 'q':
               printf("exit.\n");
               tcsetattr( handle, TCSANOW, &oldtio ); // 이전 상태로 되돌린다. 
               close( handle );   // 화일을 닫는다.               
               exit(0);
               printf("%c send\n", Buff[0]);
               break;

            default :
               printf("%d :: Wrong key ..... try again\n", key);
               break;
        }
    }
    
    tcsetattr( handle, TCSANOW, &oldtio ); // 이전 상태로 되돌린다. 
    
    close( handle );   // 화일을 닫는다.
    
    return 0;    
}

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}
 
int main_menu(void)
{
    int key;

    printf("\n\n");
    printf("-------------------------------------------------\n");
    printf("                    MAIN MENU\n");
    printf("-------------------------------------------------\n");
    printf(" 1. apple                                        \n");
    printf(" 2. person(banana)                               \n");
    printf(" 3. bicycle                                      \n");
    printf(" 4. dog                                          \n");
    printf(" 5. truck                                        \n");

    printf(" a. Turn Left                                    \n");
    printf(" b. Turn Right                                   \n");
    printf(" c. Forward                                      \n");
    printf(" d. backward                                     \n");
    printf(" i. stop                                         \n");
    printf(" I. speed up +10                                 \n");
    printf(" D. speed down -10                               \n");


    printf("-------------------------------------------------\n");
    printf(" q. Motor Control application QUIT               \n");
    printf("-------------------------------------------------\n");
    printf("\n\n");
 
    printf("SELECT THE COMMAND NUMBER : ");

    key=getch();
 
    return key;
}

