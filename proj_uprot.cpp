#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <queue>
#include <pthread.h>
#include "proj_uport.hpp"

void* UPORT_GET(void * ptr);
void* UPORT_SEND(void * ptr);

class uportDev{
public:
    uportDev()
    {
        uPort=open(UPROT_DEVICE_PATH,O_RDWR | O_NOCTTY | O_NDELAY);
        if(uPort < 0){
            printf("%s open faild\r\n",UPROT_DEVICE_PATH);
        }
        ter_s.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        ter_s.c_cflag |= CLOCAL | CREAD;//激活本地连接与接受使能
        ter_s.c_cflag &= ~CSIZE;//失能数据位屏蔽
        ter_s.c_cflag |= CS8;//8位数据
        ter_s.c_cflag &= ~CSTOPB;//1位停止位
        ter_s.c_cflag &= ~PARENB;//无校验位
        ter_s.c_cc[VTIME] = 1;//设置接受收等待超时时间 0.1s
        ter_s.c_cc[VMIN] = 0;//设置期望一次接收字节数量
        ter_s.c_oflag &= ~OPOST;
        cfsetispeed(&ter_s,B115200);//设置输入波特率
        cfsetospeed(&ter_s,B115200);//设置输出波特率
        if(tcsetattr(uPort ,TCSANOW,&ter_s) != 0)
        {
              printf("com set error!\r\n");
        }

    }
    void initialTrans()
    {
        threadStart = true;
        ExitSignel =false;
        pthread_create(&rxThread,NULL,UPORT_GET,this);
        pthread_create(&txThread,NULL,UPORT_SEND,this);
    }

    ~uportDev()
    {
        ExitSignel = true;
        if(threadStart)
        {
            pthread_join(rxThread,nullptr);
            pthread_join(txThread,nullptr);
        }
        if(uPort>0)
        {
            close(uPort);
        }

    }
    void sendData(transData* tempData)
    {
        pthread_mutex_lock(&uz_tx_mutex);
        if(uz_txdata_queue.size()<20)
        {
            uz_txdata_queue.push(*tempData);
        }
        else
        {
            printf("%d: uz_txdata_queue has full",uPort);
        }
        pthread_cond_signal(&uz_tx_cond);
        pthread_mutex_unlock(&uz_tx_mutex);
    }
    void getData(transData* tempData)
    {
        pthread_mutex_lock(&uz_rx_mutex);
        // 判断队列是否为空
        while (uz_rxdata_queue.empty()) {
            // 等待信号
            pthread_cond_wait(&uz_rx_cond, &uz_rx_mutex);
        }

        // 从队列中取出数据
        *tempData = uz_rxdata_queue.front();
        uz_rxdata_queue.pop();
        // 解锁
        pthread_mutex_unlock(&uz_rx_mutex);
    }
    int getUportHandle()
    {
        return uPort;
    }
    std::queue<transData> uz_txdata_queue;
    pthread_mutex_t uz_tx_mutex;
    pthread_cond_t uz_tx_cond;
    std::queue<transData> uz_rxdata_queue;
    pthread_mutex_t uz_rx_mutex;
    pthread_cond_t uz_rx_cond;
    bool ExitSignel=false;

private:
    int uPort;
    bool threadStart=false;
    struct termios ter_s;
    pthread_t rxThread,txThread;
};
std::vector<uportDev> uz_uPortDevice;





void* UPORT_GET(void * ptr)
{
    uportDev * objectPtr =(uportDev *)ptr;
    int len=0;

    transData tempRx;
    timeval time;
    int fs_sel=0;
    fd_set fs_read;
    while(!objectPtr->ExitSignel)
    {
        FD_ZERO(&fs_read);
        FD_SET(objectPtr->getUportHandle(),&fs_read);
        time.tv_sec=0;
        time.tv_usec=300000;
        fs_sel=select(objectPtr->getUportHandle()+1,&fs_read,NULL,NULL,&time);
        if(fs_sel)
        {
            len =read(objectPtr->getUportHandle(),tempRx.Data,UPROT_RX_DATA);
            if(len>0)
            {
                pthread_mutex_lock(&objectPtr->uz_rx_mutex);
                // 将数据加入队列
                objectPtr->uz_rxdata_queue.push(tempRx);
                // 发送信号通知消费者线程
                pthread_cond_signal(&objectPtr->uz_rx_cond);

                pthread_mutex_lock(&objectPtr->uz_rx_mutex);
            }
            else
            {
                usleep(200000);
            }
        }

    }
    return NULL;
}
void* UPORT_SEND(void * ptr)
{
    transData tempTx;
    uportDev * objectPtr =(uportDev *)ptr;
    while (!objectPtr->ExitSignel) {
        pthread_mutex_lock(&objectPtr->uz_tx_mutex);

        // 判断队列是否为空
        while (objectPtr->uz_txdata_queue.empty()) {
            // 等待信号
            pthread_cond_wait(&objectPtr->uz_tx_cond, &objectPtr->uz_tx_mutex);
        }

        // 从队列中取出数据
        tempTx = objectPtr->uz_txdata_queue.front();
        objectPtr->uz_txdata_queue.pop();

        // 解锁
        pthread_mutex_unlock(&objectPtr->uz_tx_mutex);
        //将txData内的数据写入
        write(objectPtr->getUportHandle(),tempTx.Data,sizeof(tempTx.Data)-1);
    }
    return NULL;
}


unsigned int uz_initialAUportDevice()
{
    uz_uPortDevice.push_back(uportDev());
    return uz_uPortDevice[uz_uPortDevice.size()-1].getUportHandle();
}
void uz_deInitialUportDevice(int handle)
{
    for(unsigned long count=0;count<uz_uPortDevice.size();count++)
    {
        if(uz_uPortDevice[count].getUportHandle()==handle)
        {
            uz_uPortDevice.erase(uz_uPortDevice.begin()+count);
        }
    }

}
void uz_startTrans(int handle)
{
    for(unsigned long count=0;count<uz_uPortDevice.size();count++)
    {
        if(uz_uPortDevice[count].getUportHandle()==handle)
        {
            uz_uPortDevice[count].initialTrans();
        }
    }
}
void uz_sendData(int handle,transData * input)
{
    for(unsigned long count=0;count<uz_uPortDevice.size();count++)
    {
        if(uz_uPortDevice[count].getUportHandle()==handle)
        {
            uz_uPortDevice[count].sendData(input);
        }
    }
}

void uz_getData(int handle,transData * input)
{
    for(unsigned long count=0;count<uz_uPortDevice.size();count++)
    {
        if(uz_uPortDevice[count].getUportHandle()==handle)
        {
            uz_uPortDevice[count].getData(input);
        }
    }
}

