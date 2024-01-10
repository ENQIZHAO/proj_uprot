#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <queue>
#include <pthread.h>
#include <map>
#include <termios.h>
#include <atomic>
#include "proj_uport.hpp"

void* UPORT_GET(void * ptr);
void* UPORT_SEND(void * ptr);


class uportDev{
public:
    uportDev()
    {
        ter_s.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        ter_s.c_cflag |= CLOCAL | CREAD;//激活本地连接与接受使能
        ter_s.c_cflag &= ~CSIZE;//失能数据位屏蔽
        ter_s.c_cflag |= CS8;//8位数据
        ter_s.c_cflag &= ~CSTOPB;//1位停止位
        ter_s.c_cflag &= ~PARENB;//无校验位
        ter_s.c_cc[VTIME] = 1;//设置接受收等待超时时间 0.1s
        ter_s.c_cc[VMIN] = 0;//设置期望一次接收字节数量
        ter_s.c_oflag &= ~OPOST;
        uPort=-1;
        threadExitSignel=true;
        threadStart = false;
    }
    uportDev(const uportDev &object)
    {
        this->ter_s=object.ter_s;
        this->uPort=object.uPort;
        this->threadExitSignel=object.threadExitSignel;
        this->threadStart=object.threadStart;
    }
    void openDev(const char * devicePath)
    {
        uPort=open(devicePath,O_RDWR | O_NOCTTY | O_NDELAY);
        if(uPort < 0){
            printf("%s open faild\r\n",UPROT_DEVICE_PATH);
        }
        cfsetispeed(&ter_s,B115200);//设置输入波特率
        cfsetospeed(&ter_s,B115200);//设置输出波特率
        if(tcsetattr(uPort ,TCSANOW,&ter_s) != 0)
        {
              printf("com set error!\r\n");
        }
    }

    bool setArg(unsigned int iflag,unsigned int cflag,unsigned int oflag,uz_bitRate bitrate)
    {
        ter_s.c_iflag=iflag;
        ter_s.c_oflag=oflag;
        ter_s.c_cflag=cflag;
        int bitInt=0;
        switch (bitrate) {
            case BITRATE57600:{
                bitInt = B57600;
                break;
            }
            case BITRATE115200:{
                bitInt = B115200;
                break;
            }
            case BITRATE230400:{
                bitInt = B230400;
                break;
            }
            case BITRATE460800:{
                bitInt = B460800;
                break;
            }
            case BITRATE500000:{
                bitInt = B500000;
                break;
            }
            case BITRATE576000:{
                bitInt = B576000;
                break;
            }
            case BITRATE921600:{
                bitInt = B921600;
                break;
            }
            case BITRATE1000000:{
                bitInt = B1000000;
                break;
            }
            case BITRATE1152000:{
                bitInt = B1152000;
                break;
            }
            case BITRATE1500000:{
                bitInt = B1500000;
                break;
            }
            case BITRATE2000000:{
                bitInt = B2000000;
                break;
            }
            case BITRATE2500000:{
                bitInt = B2500000;
                break;
            }
            case BITRATE3000000:{
                bitInt = B3000000;
                break;
            }
            case BITRATE3500000:{
                bitInt = B3500000;
                break;
            }
            case BITRATE4000000:{
                bitInt = B4000000;
                break;
            }
        }
        cfsetispeed(&ter_s,bitInt);//设置输入波特率
        cfsetospeed(&ter_s,bitInt);//设置输出波特率
        if(tcsetattr(uPort ,TCSANOW,&ter_s) != 0)
        {
              printf("com set error!\r\n");
              return false;
        }
        return true;
    }
    void initialTrans()
    {
        if(!threadStart)
        {
            threadExitSignel = false;
            pthread_create(&rxThread,nullptr,UPORT_GET,this);
            pthread_create(&txThread,nullptr,UPORT_SEND,this);
            threadStart = true;
        }
    }
    void deInitTrans()
    {
        if(threadStart)
        {
            threadExitSignel = true;
            pthread_cond_broadcast(&uz_tx_cond);
            pthread_cond_broadcast(&uz_rx_cond);
            pthread_join(rxThread,nullptr);
            pthread_join(txThread,nullptr);
            threadStart = false;
        }
    }
    void closeDev()
    {
        if(uPort>0)
        {
            close(uPort);
        }
    }
    ~uportDev()
    {
        deInitTrans();
        closeDev();
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
    bool threadExitSignel;

private:
    int uPort;
    bool threadStart=false;
    struct termios ter_s;
    pthread_t rxThread,txThread;
};
std::vector<uportDev> uz_uPortDevice;
std::map<std::string,uportDev> uz_uPortDevices;




void* UPORT_GET(void * ptr)
{
    if(ptr==nullptr)
    {
        return nullptr;
    }

    uportDev * objectPtr =(uportDev *)ptr;
    if(objectPtr->getUportHandle()<0)
    {
        return nullptr;
    }

    int len=0;

    transData tempRx;
    timeval time;
    int fs_sel=0;
    fd_set fs_read;
    while(!objectPtr->threadExitSignel)
    {
        FD_ZERO(&fs_read);
        FD_SET(objectPtr->getUportHandle(),&fs_read);
        time.tv_sec=0;
        time.tv_usec=300000;
        fs_sel=select(objectPtr->getUportHandle()+1,&fs_read,nullptr,nullptr,&time);
        if(fs_sel)
        {
            len =read(objectPtr->getUportHandle(),tempRx.Data,UPROT_RX_DATA);
            tempRx.dataLens=len;
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
    printf("[%s]-%d: ", __FUNCTION__, __LINE__);
    printf(" exit Report \n ");
    return nullptr;
}
void* UPORT_SEND(void * ptr)
{

    if(ptr==nullptr)
    {
        return nullptr;
    }

    uportDev * objectPtr =(uportDev *)ptr;
    if(objectPtr->getUportHandle()<0)
    {
        return nullptr;
    }
    transData tempTx;
    while (!objectPtr->threadExitSignel) {
        pthread_mutex_lock(&objectPtr->uz_tx_mutex);

        // 判断队列是否为空
        while (objectPtr->uz_txdata_queue.empty()&&!objectPtr->threadExitSignel) {
            // 等待信号
            pthread_cond_wait(&objectPtr->uz_tx_cond, &objectPtr->uz_tx_mutex);
        }

        if(!objectPtr->threadExitSignel)
        {
            // 从队列中取出数据
            tempTx = objectPtr->uz_txdata_queue.front();
            objectPtr->uz_txdata_queue.pop();
            // 解锁
            pthread_mutex_unlock(&objectPtr->uz_tx_mutex);
            //将txData内的数据写入
            printf("signel \n");
            write(objectPtr->getUportHandle(),tempTx.Data,tempTx.dataLens);
        }
        else
        {
            //退出前清空队列
            while (!objectPtr->uz_txdata_queue.empty()) {
                objectPtr->uz_txdata_queue.pop();
            }
            // 解锁
            pthread_mutex_unlock(&objectPtr->uz_tx_mutex);
        }

    }
    printf("[%s]-%d: ", __FUNCTION__, __LINE__);
    printf(" exit Report \n ");
    return nullptr;
}


unsigned int uz_initialAUportDevice(const char * devicePath)
{
    uz_uPortDevices.emplace(devicePath,uportDev());
    uportDev * ptr = &uz_uPortDevices[devicePath];
    ptr->openDev(devicePath);
    return ptr->getUportHandle();
}
void uz_deInitialUportDevice(const char * devicePath)
{
    uz_uPortDevices.erase(devicePath);
}
void uz_startTrans(const char * devicePath)
{
    uz_uPortDevices[devicePath].initialTrans();
}
void uz_stopTrans(const char * devicePath)
{
    uz_uPortDevices[devicePath].deInitTrans();
}
void uz_setPortArg(const char * devicePath,unsigned int iflag,unsigned int cflag,unsigned int oflag,uz_bitRate bitrate)
{
    uportDev * ptr = &uz_uPortDevices[devicePath];
    if(ptr->threadExitSignel)
    {
        ptr->setArg(iflag,cflag,oflag,bitrate);
    }
    else
    {
        ptr->deInitTrans();
        ptr->setArg(iflag,cflag,oflag,bitrate);
        ptr->initialTrans();
    }
}
void uz_sendData(const char * devicePath,transData * input)
{
    uz_uPortDevices[devicePath].sendData(input);
}
void uz_getData(const char * devicePath,transData * input)
{
    uz_uPortDevices[devicePath].getData(input);
}

