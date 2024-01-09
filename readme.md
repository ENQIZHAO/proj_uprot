# 说明
这是一个对串口多线程收发的程序
# 使用方法
## 建立一个串口收发通讯
```c
//初始化串口(设备路径devicePath)，返回打开的串口设备handle，注意保存好该handle，后续操作均基于该handle。
unsigned int uz_initialAUportDevice(const char * devicePath);
```
## 开始后台收发服务
```c
//handle为已经初始化好的串口handle，要想正常使用uz_sendData和uz_getData，必须先调用此函数
void uz_startTrans(int handle);
```
## 发送一组数据
```c
//handle为已经初始化好的串口handle,transData 为结构体，声明在proj_uport.hpp内，存储要发送的数据
void uz_sendData(int handle,transData * input);
```
## 接收一组数据（注意为阻塞等待）
```c
//handle为已经初始化好的串口handle,transData 为结构体，声明在proj_uport.hpp内，调用后接受数据返回在transData内。
void uz_getData(int handle,transData * input);
```
## 反初始化串口收发（关闭设备，清除内存，回收线程）
```c
//handle为已经初始化好的串口handle，函数执行回收操作。
//在程序退出前，由于类的析构函数会自动调用，不用担心程序退出后的回收问题
//该函数用于主动管理系统资源。
void uz_deInitialUportDevice(int handle);
```
## 传输结构
```c
typedef struct{
    char Data[UPROT_RX_DATA]; //数据区，数据区长度只有重新构建时可以通过宏UPROT_RX_DATA修改
    unsigned long int dataLens; //有效数据长度
}transData;
```
