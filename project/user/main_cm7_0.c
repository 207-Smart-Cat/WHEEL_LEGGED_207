/*********************************************************************************************************************
 * CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main_cm7_0
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 CMSIS-DAP 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 使用 USB-TTL 模块连接
//      模块管脚            单片机管脚
//      USB-TTL-RX          查看 zf_common_debug.h 文件中 DEBUG_UART_TX_PIN 宏定义的引脚 默认 P14_0
//      USB-TTL-TX          查看 zf_common_debug.h 文件中 DEBUG_UART_RX_PIN 宏定义的引脚 默认 P14_1
//      USB-TTL-GND         核心板电源地 GND
//      USB-TTL-3V3         核心板 3V3 电源
// 接入 imu963ra
//      模块管脚            单片机管脚
//      SCL/SPC             查看 zf_device_imu963ra.h 中 imu963ra_SPC_PIN 宏定义 默认 P20_11
//      SDA/DSI             查看 zf_device_imu963ra.h 中 imu963ra_SDI_PIN 宏定义 默认 P20_14
//      SA0/SDO             查看 zf_device_imu963ra.h 中 imu963ra_SDO_PIN 宏定义 默认 P20_12
//      CS                  查看 zf_device_imu963ra.h 中 imu963ra_CS_PIN  宏定义 默认 P20_13
//      GND                 电源地 GND
//      3V3                 电源 3V3

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，并连接好编码器，在断电情况下完成连接
// 2.将调试下载器或者 USB-TTL 模块连接电脑 完成上电 正常 H2 LED 会闪烁
// 3.电脑上使用 逐飞助手 打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.可以在 逐飞助手 上看到如下串口信息：
//      imu963ra acc data: x-..., y-..., z-...
//      imu963ra gyro data: x-..., y-..., z-...
//      imu963ra mag data: x-..., y-..., z-...
// 5.移动旋转 imu963ra 就会看到数值变化
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
#define LED1 (P19_0) // SPI 串口 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_init();                  // 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式

    while (1)
    {
        if (imu963ra_init())
        {
            printf("\r\n imu963ra init error."); // imu963ra 初始化失败
        }
        else
        {
            break;
        }
        gpio_toggle_level(LED1); // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }

    // 此处编写用户代码 例如外设初始化代码等
    while (true)
    {
        // 此处编写需要循环执行的代码
        imu963ra_get_acc();  // 获取 imu963ra 的加速度测量数值
        imu963ra_get_gyro(); // 获取 imu963ra 的角速度测量数值
       // imu963ra_get_mag();  // 获取 IMU963RA 的地磁计测量数值
        printf("imu963ra acc data: %d,%d,%d,%d,%d,%d\n", imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z,imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z);

       
       // printf("\r\nimu963ra gyro data: %d,%d,%d\n", imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z);
        // system_delay_ms(50);
        // printf("\r\nimu963ra mag data:  %d,%d,%d\n", imu963ra_mag_x, imu963ra_mag_y, imu963ra_mag_z);
        system_delay_ms(50);
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看 逐飞助手 打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 CMSIS-DAP 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看 逐飞助手 设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：串口输出 imu963ra init error.
//      检查imu963ra的接线是否正确
//      检查imu963ra的模块是不是坏了
//      给信号线加上拉看看
// 问题4：imu963ra 数值异常
//      看看是不是线松了 或者信号线被短路了
//      可能模块部分受损