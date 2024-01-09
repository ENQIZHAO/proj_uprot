#include <sys/types.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define UPROT_DEVICE_PATH ""
#define UPROT_RX_DATA 128


void* UPORT_GET_ORDER()
{
    int uPort=0,len=0;
    char rxData[UPROT_RX_DATA]={0};
    char txData[UPROT_RX_DATA]={0};
    struct timeval time;


    uPort=open(UPROT_DEVICE_PATH,O_RDWR | O_NOCTTY | O_NDELAY);
    if(uPort < 0){
        printf("%s open faild\r\n",UPROT_DEVICE_PATH);
    }
    struct termios ter_s = {0};

      ter_s.c_iflag &= ~(INLCR | ICRNL | IGNCR),
      ter_s.c_cflag |= CLOCAL | CREAD,//激活本地连接与接受使能
      ter_s.c_cflag &= ~CSIZE,//失能数据位屏蔽
      ter_s.c_cflag |= CS8,//8位数据
      ter_s.c_cflag &= ~CSTOPB,//1位停止位
      ter_s.c_cflag &= ~PARENB,//无校验位
      ter_s.c_cc[VTIME] = 1,//设置接受收等待超时时间 0.1s
      ter_s.c_cc[VMIN] = 0,//设置期望一次接收字节数量
      ter_s.c_oflag &= ~OPOST,

    cfsetispeed(&ter_s,B115200);//设置输入波特率
    cfsetospeed(&ter_s,B115200);//设置输出波特率
    if(tcsetattr(uPort,TCSANOW,&ter_s) != 0)
      {
            printf("com set error!\r\n");
            return NULL;
      }

    int fs_sel=0;
    fd_set fs_read;
    while(1)
    {
        FD_ZERO(&fs_read);
        FD_SET(uPort,&fs_read);
        time.tv_sec=0;
        time.tv_usec=200000;
        fs_sel=select(uPort+1,&fs_read,NULL,NULL,&time);
        if(fs_sel)
        {
            len =read(uPort,rxData,UPROT_RX_DATA);
            if(len>0)
            {
                //raData内是接收到的数据，
                //len是数据长度，字节
            }
            else
            {
                usleep(200000);
            }
        }
        //将txData内的数据写入
        write(uPort,txData,sizeof(txData)-1);
    }
    return NULL;
}
