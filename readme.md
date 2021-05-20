主控芯片拟采用OpenIOE AMC。

官方介绍：

> 符合官方设计的MicroPython 控制器板，完全支持软件功能。 硬件：
>
> - 美国原装进口 STM32F765VIT6 微控制器
> - 216 MHz
> - 512KB RAM
> - 微型USB连接器，用于电源和串行通信
> - SPI
> - UART
> - IIC
> - ROM中的DFU引导加载程序，便于升级固件

![OpenIOE AMC](http://lingread.celerstar.com/php/uploads/20180106165100gDJ9j9EIHNFA035wpinout.jpg)

官方网站：[MicroPython 中文网 (openioe.net)](http://micropython.openioe.net/)

参考文档：[OpenIOE-智能硬件](http://www.openioe.net/amc_listlingread.html?qrcodeIotPID=gh941SnHkjW75IJM&qrcodeDatasId=1141&id=1444)

> 使用该开发板的原因:
>
> - [x] 支持micropython开发，矩阵运算，图像分析等计算需求得到满足
>
> - [x] 有许多强大的函数库供使用，编程易上手
> - [x] 采用STM32V7系列，性能十分强大
> - [ ] ~~价格昂贵~~

对于板载资源分配：

| I2C1    | I2C2     | UART2  |
| ------- | -------- | ------ |
| AMG8833 | VEML6075 | JY901S |

AMG8833接线：（红外热像传感器）

| AMG8833 | GND  | VCC  | SCL  | SDA  |
| ------- | ---- | ---- | ---- | ---- |
| OpenIOE | GND  | 5V   | E04  | E03  |

VEML6075接线：（紫外线传感器）

| VEML6075 | GND  | VCC  | SCL  | SDA  |
| -------- | ---- | ---- | ---- | ---- |
| OpenIOE  | GND  | 3.3V | E13  | E14  |

JY901S接线：（九轴陀螺仪）

| JY901S  | GND  | TX   | RX   | VCC  |
| ------- | ---- | ---- | ---- | ---- |
| OpenIOE | GND  | W05  | W04  | 5V   |

JY901S使用GPS功能需要连接到GPS模块，选择同家公司生产的WTGPS-200与其连接。

| WTGPS-200 | GND  | TXD  | RXD  | VCC  |
| --------- | ---- | ---- | ---- | ---- |
| JY901S    | GND  | D1   | D2   | VCC  |

AMG8833参数：

![22](https://www.hualigs.cn/image/60a4b94aac469.jpg)

![amg8833参数2](https://i0.hdslb.com/bfs/album/9a364e9a2cbe883d6e71f0d65fbc8b88fbbafbec.jpg)

VEML6075参数：

![mpu9250参数](https://i0.hdslb.com/bfs/album/b076660cf4b2c273234a9d6d2786eed1d2f9ef24.jpg)

JY901S参数：

![veml6075参数](https://i0.hdslb.com/bfs/album/b50046ff6247ffe9d8b8232455cca13eb97e87d5.jpg)

WTGPS-200参数：

![wtgpw-200参数](https://www.hualigs.cn/image/60a4b892e31d8.jpg)

代码部分：

AMG8833驱动代码：

```python
from pyb import I2C
address = 0x69
amg8833 = I2C(1, i2C.MASTER)
amg8833.mem_write(0x00, 0x08)#enter normal mode
amg8833.mem_write(0x30, 0x08)#software reset
amg8833.mem_write(0x00, 0x03)#disable interrupts by default
amg8833.mem_write(0x00, 0x01)#set to 10 FPS

def readPixels():
    amg8833.mem_write(0xad,address)#发送读取指令
    for i in range(0,63):
        buf[i] = amg8833.mem_read(1, address, 0x30+i)
        #从amg8833读取储存器0x30~0x69总计64个像素数据并保存到buf数组中
    return buf
```

参考链接：[github:Adafruit_AMG88xx](https://github.com/adafruit/Adafruit_AMG88xx)

由于AMG8833热成像像素低，只有8*8，故需要对其进行处理，有双线性插值法公式如下：

$\Large F_\alpha(i+u,j+v)=(1-u)(1-v)F(i,j)+v(1-u)F(i,j+1)+u(1-v)F(i+1,j)+uvF(i,j)$

参考链接：[bilibili:图像双线性插值处理](https://www.bilibili.com/read/cv10879872/)

```python
import numpy as np
def interpolation(A, i, j, u, v):
    A[2*(i+u),2*(j+v)] = (1-u)*(1-v)*A[i,j]\
                        +(1-u)*(v)*A[i,j+1]\
                        +(u)*(1-v)*A[i+1,j]\
                        +(u)*(v)*A[i+1,j+1]
A = np.asarray(buf).reshape(8,8)
#将数组重整为8*8矩阵,buf为上文从AMG8833接收的总计64个元素
B = np.matlib.zeros((16,16))
#创建一个16*16的零矩阵
for i in range(0,7):
    for j in range(0,7):
        B[2*i, 2*j] = A[i, j];
for i in range(0,7):
    for j in range(0,7):
            interpolation(B, i, j, 0.5, 0)
            interpolation(B, i, j, 0, 0.5)
            interpolation(B, i, j, 0.5, 0.5)
#16*16的B矩阵256个元素全部被赋值完毕
#重复上述操作两次，形成一个64*64矩阵
C = np.matlib.zeros((32,32))
for i in range(0,15):
    for j in range(0,15):
        C[2*i, 2*j] = B[i, j];
for i in range(0,15):
    for j in range(0,15):
            interpolation(C, i, j, 0.5, 0)
            interpolation(C, i, j, 0, 0.5)
            interpolation(C, i, j, 0.5, 0.5)
D = np.matlib.zeros((64,64))
for i in range(0,31):
    for j in range(0,31):
        D[2*i, 2*j] = C[i, j];
for i in range(0,31):
    for j in range(0,31):
            interpolation(D, i, j, 0.5, 0)
            interpolation(D, i, j, 0, 0.5)
            interpolation(D, i, j, 0.5, 0.5)
#通过插值完成8*8矩阵转换到64*64高分辨率矩阵
```

![效果图](https://img-blog.csdnimg.cn/20210510232042684.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80Mzc5NjU5Mw==,size_16,color_FFFFFF,t_70#pic_center)

<p align = 'right'>  效果图      </p> 

通过采集到的数据进行图像处理，提取其特性例如：圆形度，面积变化率，相关特性，边缘抖动率和闪烁频率判断热成像高温像素区域是否为火焰元素。

面积变化率：

```python
temperature_threshold = 78#  AMG8833的热成像范围是—20°C至80°C，森林中火情点不充分燃烧时中心点至少400°C，距离远时78°C可作为一个温度判断阈值
def dimen():#求得面积
    E = np.dot(1/78, D)  #D为上文64*64矩阵，与阈值的倒数点乘，高于78°C的像素点大于1，小于78°C的像素点小于1
    E = np.floor(E)  #E矩阵向下取整,高温像素为1，低温像素为0，对矩阵各元素求和即为面积
    return numpy.sum(E) #返回E矩阵总和，即为高温像素面积
#再将其在定时器内运算得到面积变化率，此处不再作阐述
```

圆形度与边缘抖动率：

```python
def edge():#边缘检测
	fliter = np.mat('-1 -1 -1; -1 8 -1; -1 -1 -1')
    """
    创建一个检测所有方向的边缘检测核半径为1滤波器
    -1 -1 -1
    -1  8 -1
    -1 -1 -1
    """
    F = np.matlib.zeros((62,62))#与滤波器卷积后损失64*64-(64-2)*(64-2)的像素，新矩阵为62*62
	for i in range(0, 62):
    	for j in range(0, 62):
        	F[i,j] = numpy.vdot(E[i:i+2,j:j+2], fliter)
            #将滤波器与D矩阵卷积
    return F
#得到滤波后的边缘热成像图后，作霍夫圆检测或拟合圆，最小外接圆可得到圆形度的数据
#对多次检测得到的图形拟合在一起，求其梯度可得到边缘抖动率，后面涉及到太多机器学习与图像处理的技术我不会就不再丢人现眼了
```

参考链接：[知乎：图像处理基本知识](https://zhuanlan.zhihu.com/p/55013828)

判断为火焰元素后，需要配合紫外线传感器进行综合比较，VEML6075驱动代码如下：

```python
from pyb import I2C
_address = 0x26
veml6075 = I2C(2, i2C.MASTER)
def readValue():
    veml6075.mem.write(0x01, _address)
    buf[0] = veml6075.mem.read(1, _address, 0x18)#读取uva
    veml6075.mem.write(0x02, _address)
    buf[1] = veml6075.mem.read(1, _address, 0x19)#读取uvb
    veml6075.mem.write(0x02, _address)
    buf[2] = veml6075.mem.read(1, _address, 0x1A)#读取uvi
    return buf
```

参考链接：[github:SparkFun_VEML6075_Arduino_Library](https://github.com/sparkfun/SparkFun_VEML6075_Arduino_Library)

> uva,uvb均为紫外线波长划分的一部分,另有uvc该传感器未能测出，uvi为红外线指标，与火焰检测无关。
>
> uva波长为320~420nm，属长波
>
> uvb波长为290~320nm，属中波
>
> uvc波长为100~290nm，属短波

参考链接：[百度知道：紫外线Uva,Uvb,Uvc代表什么意思](https://zhidao.baidu.com/question/2055098205535756427.html)

当红外热像传感器检测到火焰元素时，紫外线传感器开始检测火焰紫外线。

火焰的辐射是具有离散光谱的气体辐射和伴有连续光谱的固体辐射，其波长在0.1-10μm或更宽的范围，为了避免其他信号的干扰，常利用==波长<340nm的紫外线==，或者火焰中特有的波长在4.4μm附近的CO2辐射光谱作为探测信号。

到达大气层下地面的太阳光和非透紫材料作为玻壳的电光源发出的光波长均大于340nm，故火焰探测的220m-320nm中紫外波段属太阳光谱盲区（日盲区）。紫外火焰探测技术，使系统避开了最强大的自然光源一太阳造成的复杂背景，使得在系统中信息处理的负担大为减轻。所以可靠性较高，加之它是光子检测手段，因而信噪比高，具有极微弱信号检测能力，除此之外，它还具有反应时间极快的特点。

参考链接：[海川化工论坛：火焰探测的基本原理](https://bbs.hcbbs.com/thread-751949-1-1.html)

故选择uvb数据作为判断红外热像疑似火焰元素是否为火情点，当uvc数据远远大于正常情况下的最大数值UvcT且一段时间内不恢复时，即认为无人机检测到火情点。

无人机需要在发现火情点后，机载云台对准火情点中心，因此需要高精度陀螺仪进行姿态矫正，PID计算控制无人机机载云台飞方向对准火情点，同时需要快速返回该地的经纬度信息。

JY901S驱动代码如下：

```python
from pyb import UART
mpu9250 = uart(2,115200)
mpu9250.init(115200, bits=8, parity=None, stop=1)#mpu9250默认波特率9600，速度太慢，故需要先将mpu9250连接到电脑上位机将其波特率调为115200
def set_back(ti=flase, ac=flase, av=flase, ag=flase, mg=flase, pt=flase, \
             ps_hi=flase, lo_la=flase, es=flase, _4q=flase, sl=flase):
    #依次为：时间，加速度，角速度，角度，磁场，端口状态，气压和高度，经纬度，地速，四元素,卫星定位精度
    mpu9250.write(0x00000<<12|sl<<11|_4q<<10|es<<9|lo_la<<8|ps_hi<<7|pt<<6|mg<<5|ag<<4|ag<<3|av<<2|ac<<1|ti)
    
set_back(ac=ture, av=true, ag=true, ps_hi=true, lo_la=true, _4q=true)#只需要加速度，角速度，角度，高度，经纬度，四元素的数据

def ac_cal(buf):#加速度
    ax=ay=az=0  #防止检验出错导致返回出错进而导致程序崩溃
    if buf[0]==0x55 and buf[1]==0x51:
        for i in range(0,9)
        	sum += buf[i]
    	if sum == buf[10]:
            ax = buf[2]<<8|buf[3]/32786*16*9.8
            ay = buf[4]<<8|buf[5]/32786*16*9.8
            az = buf[6]<<8|buf[7]/32786*16*9.8
    return (ax,ay,az)

def av_cal(buf):#角速度
    wx=wy=wz=0
    if buf[0]==0x55 and buf[1]==0x52:
        for i in range(0,9)
        	sum += buf[i]
    	if sum == buf[10]:
            wx = buf[2]<<8|buf[3]/32786*2000
            wy = buf[4]<<8|buf[5]/32786*2000
            wz = buf[6]<<8|buf[7]/32786*2000
    return (wx,wy,wz)

def ag_cal(buf):#角度
    px=py=pz=0
    if buf[0]==0x55 and buf[1]==0x53:
        for i in range(0,9)
        	sum += buf[i]
    	if sum == buf[10]:
            px = buf[2]<<8|buf[3]/32786*180
            py = buf[4]<<8|buf[5]/32786*180
            pz = buf[6]<<8|buf[7]/32786*180
    return (px,py,pz)

def ps_hi_cal(buf)#高度
	ps=hi=0
	if buf[0]==0x55 and buf[1]==0x56:
        for i in range(0,9)
        	sum += buf[i]
    	if sum == buf[10]:
            ps = buf[5]<<24|buf[4]<<16|buf[3]<<8|buf[2]
            hi = buf[9]<<24|buf[8]<<16|buf[7]<<8|buf[6]
    return (ps,hi)

def lo_la_cal(buf):#经纬度
    Lon=Lat=0
    if buf[0]==0x55 and buf[1]==0x57:
        for i in range(0,9)
        	sum += buf[i]
    	if sum == buf[10]:
            lon = buf[5]<<24|buf[4]<<16|buf[3]<<8|buf[2]
            dd = lon/100000000
            mm = (lon%100000000)
            Lon = dd+mm #经度
            lat = buf[9]<<24|buf[8]<<16|buf[7]<<8|buf[6]
            dd = lat/100000000
            mm = (lat%100000000)
            Lat = dd+mm #维度
    return (Lon, Lat)

def _4q_cal(buf)
	q0=q1=q2=q3=0
    if buf[0]==0x55 and buf[1]==0x59:
        for i in range(0,9)
        	sum += buf[i]
    	if sum == buf[10]:
            q0 = buf[2]<<8|buf[3]
            q1 = buf[4]<<8|buf[5]
            q2 = buf[6]<<8|buf[7]
            q3 = buf[8]<<8|buf[9]
    return (q0, q1, q2, q3)
    
def read_mpu9250():
	buf = mpu9250.readline()
	ac_cal(buf)
	av_cal(buf)
	ag_cal(buf)
	ps_hi_cal(buf)
	lo_la_cal(buf)
	_4q_cal(buf)
```

注：MPU9250为JY901S的另一名称，WTGPS-200的代码出厂时已经封装好了，不需要进行二次开发，直接连接到JY901S即可。

参考文档：[维特智能：JY901S使用说明书](http://dl.wit-motion.com:2103/index.html#/markdown/preview?document=37ec3ecfdc804a79a1fb99ea215b3913)

参考文档：[维特智能：WTGPS-200使用说明书](http://dl.wit-motion.com:2103/index.html#/markdown/preview?document=4e867c8789844962ae89f678636ad12b)

姿态解算部分：

$$\begin{cases}\Large pitch=\arcsin(2(q_0*q_2-q_1*q_3)\\
\Large roll=\arctan({2(q_0*q_2+q_1*q_3)\over q_0^2-q_1^2-q_2^2+q_3^2})\\
\Large yaw=\arctan({2(q_0*q_3+q_1*q_2)\over q_0^--q_1^2-q_2^2+q_3^2}) \end{cases}$$

程序表示为：

```python
def DMP(_4q):
    pitch = asin(-2*_4q.q1*_4q.q3+2*_4q.q0*_4q.q2)
    roll  = atan(2*_4q.q2*_4q.q3+2*_4q.q0*_4q.q1-2*_4q.q1*_4q.q1-2*_4q.q2*_4q.q2+1)
    yaw   = atan(2*(_4q.q1*q2+_4q.q0*q3)+_4q.q0*_4q.q0+_4q.q1*_4q.q1-_4q.q2*_4q.q2-_4q.q3*_4q.q3)
    return (pitch, roll, yaw)
```

参考链接：[知乎：MPU6050姿态解算方式-DMP ](https://zhuanlan.zhihu.com/p/165156300)

PID控制部分:

机载云台对准热图像中的火焰面积中心，需要求得火焰区域内的中心，因为矩阵是带有温度权重的，故可不一定要取面积中心，取重心效果更好。

对火焰元素区域求重心，参考二重积分求重心坐标有：

$$\Large \overline x=\iint_D{x\rho(x,y)d\sigma\over\rho(x,y)d\sigma}$$                               $$\Large \overline y=\iint_D{y\rho(x,y)d\sigma\over\rho(x,y)d\sigma}$$

因为矩阵是离散型数据，故需对其进行离散化处理，有处理后的求重心公式如下：

$$\Large \overline x={\sum_i^ni*\sum_j^n\rho(x_i,y_j)\over\sum_i^n\sum_j^n\rho(x_i,y_j)}$$                          $$\Large \overline y={\sum_j^nj*\sum_i^n\rho(x_i,y_j)\over\sum_i^n\sum_j^n\rho(x_i,y_j)}$$

代码实现过程：

```python
def curx():
    G = np.dot(D,E)  #过滤掉非高温像素，只留下高温火焰区域像素，D为上文进行三次双线性插值的64*64矩阵，E为上文判断是否为火焰元素的64*64矩阵
    _x = _y = 0
    for i in range(0,63):
        _x += i*np.sum(G,axis=0) #axis=0:列求和
        _y += i*np.sum(G,axis=1) #axis=1:行求和
    G_sum = np.sum(G)
    _x /= G_sum
    _y /= G_sum
    return(_x, _y)
```

因为机载云台旋转是使用三轴舵机进行旋转，选择增量式PID算法有着比位置式PID算法更稳定快速的优势，故选择增量式PID算法。

增量式PID算法如下：

$$\Large output=\sum_{i=0}^\infty P*(e_i-e_{i-1})+I*e_i+D*(e_i-2e_{i-1}+e_{i-2})$$

其中e为图像中心(32,32)与火焰元素重心(_x,_y)的误差值，PID算法通过该误差值进行角度修正，计算出output，其中x_output改变yaw轴舵机的PWM，y_output改变pitch轴舵机的PWM。

通过增量式PID控制疑似火焰元素在机载云台正前方，具体代码如下：

```python
class PID(object):
    calss Struct(object):
        def __init__(_p, _i, _d):
            self.p = _p
            self.i = _i
            self.d = _d
        def incremental_pid(p, i, d, pixels):		
        	#增量式PID,pixels为红外热像返回数据
            error_x = pixel.x - center_x
            error_y = pixel.y - center_y
            #分别返回的火焰元素中心位置和矩阵中心位置
            if error_x>0.5 : #当误差很小时，为防止抖动可停止PID计算
                output_yaw += (error_x-last_error_x)*p/
                             +error_x*i /
                             +(error_x-2*last_error_x+last_last_error_x)
            if error_y>0.5:
                output_pitch += (error_y-last_error_y)*p/
                             +error_y*i /
                             +(error_y-2*last_error_y+last_last_error_y)
            last_last_error_x = last_error_x
            last_error_x = error_x
            last_last_error_y = last_error_y
            last_error_y = error_y
            return (output_yaw, output_pitch)#output反馈到云台的三轴舵机控制，使其对准疑似火焰元素
```

参考链接：[CSDN:位置式PID与增量式PID](https://blog.csdn.net/anlog/article/details/83106466)

经纬度信息部分：

选择同家公司生产的WTGPS-200与MPU9250连接，可以拓展GPS功能，发送经纬度信息给主控芯片。
