#ifndef PROJ_UPORT_HPP
#define PROJ_UPORT_HPP
#define UPROT_DEVICE_PATH ""
#define UPROT_RX_DATA 1024 //串口传输队列元素的长度，选择适合的长度
typedef struct{
    char Data[UPROT_RX_DATA];
    unsigned long int dataLens;
}transData;


#ifdef __cplusplus
extern "C" {
#endif
unsigned int uz_initialAUportDevice(const char * devicePath);
void uz_deInitialUportDevice(int handle);
void uz_startTrans(int handle);
void uz_sendData(int handle,transData * input);
//注意，该函数为阻塞方式获得数据，如果接收缓存没有数据，则阻塞等待。
void uz_getData(int handle,transData * input);
#ifdef __cplusplus
}
#endif

#endif // PROJ_UPORT_HPP
