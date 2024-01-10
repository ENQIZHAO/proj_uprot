#ifndef PROJ_UPORT_HPP
#define PROJ_UPORT_HPP
#define UPROT_DEVICE_PATH ""
#define UPROT_RX_DATA 1024 //串口传输队列元素的长度，选择适合的长度
typedef struct{
    char Data[UPROT_RX_DATA];
    unsigned long int dataLens;
}transData;
typedef enum{
    BITRATE57600  ,
    BITRATE115200 ,
    BITRATE230400 ,
    BITRATE460800 ,
    BITRATE500000 ,
    BITRATE576000 ,
    BITRATE921600 ,
    BITRATE1000000,
    BITRATE1152000,
    BITRATE1500000,
    BITRATE2000000,
    BITRATE2500000,
    BITRATE3000000,
    BITRATE3500000,
    BITRATE4000000,
}uz_bitRate;

#ifdef __cplusplus
extern "C" {
#endif
unsigned int uz_initialAUportDevice(const char * devicePath);
void uz_deInitialUportDevice(const char * devicePath);
void uz_startTrans(const char * devicePath);
void uz_sendData(const char * devicePath,transData * input);
void uz_stopTrans(const char * devicePath);
//注意，该函数为阻塞方式获得数据，如果接收缓存没有数据，则阻塞等待。
void uz_getData(const char * devicePath,transData * input);
void uz_setPortArg(const char * devicePath,unsigned int iflag,unsigned int cflag,unsigned int oflag,uz_bitRate bitrate);
#ifdef __cplusplus
}
#endif

#endif // PROJ_UPORT_HPP
