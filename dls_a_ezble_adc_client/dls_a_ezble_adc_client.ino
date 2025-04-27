#include <ezBLE.h>
#include <ArduinoLowPower.h>
#include <EEPROM.h>
#include <WatchdogTimer.h> // 引入看门狗库

// void onWatchdogOverflow();

// 数据包结构
struct DataPacket {
    uint8_t header;       // 数据头
    uint8_t length;       // 数据长度
    uint8_t data[60];     // 数据内容（最大长度为60字节）
    uint8_t checksum;     // 校验和
};

// 数据结构
struct data {
    float adc_source;
};

// 定义循环队列类
class CircularQueue {
private:
    static CircularQueue instance; // 静态单例实例
    static const int MAX_SIZE = 100; // 队列最大容量
    data queue[MAX_SIZE]; // 存储数据的静态数组
    int head; // 队列头部指针
    int tail; // 队列尾部指针
    int count; // 队列中元素的数量

    CircularQueue() { head = tail = count = 0; } // 私有构造函数，仅用于声明

public:
    // 初始化函数
    void init() {
        head = 0;
        tail = 0;
        count = 0;
    }

    // 重置队列状态
    void reset() {
        init(); // 重置时调用初始化函数
    }

    // 获取单例实例
    static CircularQueue* getInstance() {
        return &instance;
    }

    // 入队操作
    void enqueue(const data* item) {
        if (item == nullptr) {
            // Serial.println("Invalid input data.");
            return;
        }
        if (count == MAX_SIZE) {
            // 队列满了，自动出队清理
            Serial.println("Queue is full, dequeueing to make space.");
            dequeue(nullptr);
        }
        queue[tail] = *item;
        tail = (tail + 1) % MAX_SIZE;
        count++;
    }

    // 出队操作
    int dequeue(data* item) {
        if (item == nullptr && count > 0) {
            // 如果传入指针为空，仅用于清理队列
            head = (head + 1) % MAX_SIZE;
            count--;
            return 1; // 表示成功清理
        }
        if (count == 0) {
            return 0; // 队列为空，无法出队
        }
        *item = queue[head];
        head = (head + 1) % MAX_SIZE;
        count--;
        return 1; // 返回1表示成功
    }

    // 检查队列是否为空
    bool isEmpty() {
        return count == 0;
    }
};

// 初始化单例实例
CircularQueue CircularQueue::instance;

// 定义BLE配置结构体
struct BLEConfig {
    char serviceName[64]; // 服务名称，最大64字节
};

BLEConfig bleConfig; // 全局BLE配置实例

// 定义BLE通信处理结构体
struct BLEHandler {
    static BLEHandler* instance; // 单例实例指针
    String receivedData;          // 用于存储接收到的数据
    String serialData;            // 用于存储串口接收到的数据
    bool isConnected;             // 用于标记BLE设备是否连接

    // 私有构造函数，防止外部直接创建实例
    BLEHandler() {
        Serial.begin(115200);
        Serial.println("ezBLE dls client");
        Serial.println(__DATE__); // 打印编译日期
        Serial.println(__TIME__); // 打印编译时间
        // 从EEPROM读取配置
        EEPROM.get(0, bleConfig);
        if (strlen(bleConfig.serviceName) == 0) {
            strcpy(bleConfig.serviceName, "defaultService");
        }
        Serial.printf("Service name: %s\n", bleConfig.serviceName);

        isConnected = false; // 初始状态为未连接
    }

    // 获取单例实例
    static BLEHandler* getInstance() {
        if (instance == nullptr) {
            instance = new BLEHandler();
        }
        return instance;
    }

    // 初始化BLE服务
    void begin() {
        // 初始化BLE服务
        ezBLE.beginClient(bleConfig.serviceName);
        ezBLE.onReceive(onReceive); // 注册接收回调

        // 启动看门狗定时器，设置超时时间 WDOG_PERIOD_513_MS
        WatchdogTimer.begin(WDOG_PERIOD_257_MS);

        // WatchdogTimer.attachInterrupt(onWatchdogOverflow);
      
        Serial.println("Waiting for BLE connection...");
    }

    // 处理接收到的数据
    static void onReceive(int bytes) {
        (void)bytes; // 忽略bytes参数
        BLEHandler* handler = BLEHandler::getInstance(); // 获取单例实例

        while (ezBLE.available()) {
            char c = (char)ezBLE.read();
            Serial.print(c);

            // if (c == '\n') { // 如果接收到换行符
            //     // 处理接收到的完整字符串
            //     if (handler->receivedData.length() > 0) {
            //         // 假设接收到的字符串直接作为新的服务名称
            //         handler->receivedData.toCharArray(bleConfig.serviceName, sizeof(bleConfig.serviceName));
            //         // 重启设备
            //         handler->restart();
            //     }
            //     handler->receivedData = ""; // 清空接收缓冲区
            // } else {
            //     handler->receivedData += c; // 将字符追加到接收缓冲区
            // }
        }
    }

    // 重启设备
    void restart() {
        Serial.println("Restarting device...");
        // 不喂狗，让看门狗触发重启
        while (true) {
            // 故意不喂狗，等待看门狗触发重启
        }
    }

    // 发送数据
    void send(const char* data) {
        // 如果设备未连接，不执行后续操作
        if (!isConnected) {
            return;
        }
        ezBLE.printf("%s\n", data);
    }

    // 处理串口输入
    void handleSerialInput() {
        // if (Serial.available()) {
            while (Serial.available()) {
                WatchdogTimer.feed();
                char c = Serial.read();
                if (c == '\n') { // 如果接收到换行符
                    // 处理串口接收到的完整字符串
                    if (serialData.length() > 0) {
                        // 假设串口接收到的字符串直接作为新的服务名称
                        serialData.toCharArray(bleConfig.serviceName, sizeof(bleConfig.serviceName));
                        // 将新配置写入EEPROM
                        EEPROM.put(0, bleConfig);
                        // 重启设备
                        restart();
                    }
                    serialData = ""; // 清空串口接收缓冲区
                } else {
                    serialData += c; // 将字符追加到串口接收缓冲区
                }
            }
        // }
    }

    // 主循环
    void loop() {
        // 喂狗，防止看门狗触发重启
        WatchdogTimer.feed();

        // 处理串口输入
        handleSerialInput();
        // Serial.println("handleSerialInput");

        // 检查BLE设备是否连接
        if (!isConnected && ezBLE.connected()) {
            isConnected = true;
            Serial.println("BLE device connected!");
        } else if (isConnected && !ezBLE.connected()) {
            isConnected = false;
            Serial.println("BLE device disconnected. Waiting for connection...");
        }

        // 如果设备未连接，不执行后续操作
        if (!isConnected) {
            return;
        }

        // 从队列中获取数据并发送
        while (!CircularQueue::getInstance()->isEmpty()) {
            WatchdogTimer.feed();
            data adcData;
            if (CircularQueue::getInstance()->dequeue(&adcData)) {
                sendDataPacket(adcData);
            }
        }
    }

    // 序列化数据并发送
    void sendDataPacket(data adcData) {
        uint8_t dataToSend[60];
        size_t length = 0;

        // 将浮点数转换为字节数组
        memcpy(dataToSend, &adcData.adc_source, sizeof(float));
        length = sizeof(float);

        // 构造数据包
        DataPacket packet;
        packet.header = 0xA5; // 数据头
        packet.length = length;
        memcpy(packet.data, dataToSend, length);

        // 计算校验和
        uint8_t checksum = packet.header + packet.length;
        for (size_t i = 0; i < length; ++i) {
            checksum += packet.data[i];
        }
        packet.checksum = checksum;

        // 发送数据包
        ezBLE.write(packet.header);
        ezBLE.write(packet.length);
        ezBLE.write(packet.data, length);
        ezBLE.write(packet.checksum);
    }
};

// 初始化单例实例指针
BLEHandler* BLEHandler::instance = nullptr;

// 定义一个结构体来封装ADC相关的功能
struct ADCController {
    int adcPin; // ADC引脚
    int adcSamples; // 每次采集的样本数量
    int inampBridgeDriveDelay; // Inamp桥驱动延迟
    int settleTime; // 稳定时间（微秒）
    int delayTime; // 延迟时间（毫秒）

    // 构造函数，初始化参数
    ADCController(int adcPin, int adcSamples, int inampBridgeDriveDelay, int settleTime, int delayTime)
        : adcPin(adcPin), adcSamples(adcSamples), inampBridgeDriveDelay(inampBridgeDriveDelay), settleTime(settleTime), delayTime(delayTime) {}

    // 初始化函数
    void setup() {
        Serial.begin(115200); // 初始化串口通信
        pinMode(4, OUTPUT);   // Inamp enable
        pinMode(PA4, OUTPUT); // Inamp reference buffer
        pinMode(13, OUTPUT);  // Bridge driver
        pinMode(9, OUTPUT);   // ADC driver buffer
        pinMode(14, OUTPUT);  // Inamp reference buffer
    }

    // 主循环函数
    void loop() {
        // 模拟Inamp和桥驱动的控制
        digitalWrite(4, LOW); // Inamp enable
        digitalWrite(PA4, HIGH); // Inamp reference buffer
        digitalWrite(13, LOW);  // Drive bridge
        digitalWrite(9, HIGH); // Turn on ADC driver buffer

        // 静态变量用于跟踪延迟时间
        static int delay_time = 0;
        delay_time += 10;
        if (delay_time > 1000) {
            delay_time = 100;
        }

        static uint32_t tmp = 0;

        // 延迟指定时间
        delayMicroseconds(delay_time);

        // 采集ADC数据并计算平均值
        float average = readADC();

        // 恢复Inamp和桥驱动的初始状态
        digitalWrite(4, HIGH); // Inamp enable
        digitalWrite(14, LOW); // Inamp reference buffer
        digitalWrite(13, HIGH);  // Drive bridge
        digitalWrite(9, LOW); // Turn off ADC driver buffer

        // if(BLEHandler::getInstance()->isConnected)
        {
          // 打印采集到的平均值
          Serial.print("Time for ");
          Serial.print(delay_time);
          Serial.print(" us: ");
          Serial.print(tmp++);
          Serial.print(" test: ");
          Serial.println(average);
        }

        // 创建数据结构并推入队列
        data adcData;
        adcData.adc_source = average;
        CircularQueue::getInstance()->enqueue(&adcData);
        
        // 低功耗睡眠模式
        if(BLEHandler::getInstance()->isConnected)
        {
          LowPower.sleep(98);
        }
        else 
        {
          LowPower.sleep(25); // 休眠会影响串口收发，可以没配对前休眠短一些。
        }
        // 测试配置，发现不连接蓝牙的时候，内存太小会导致系统睡死恢复不了。
        // delayMicroseconds(100*1000);
        // LowPower.sleep(100);
        // 测试时发现，如果没有连上蓝牙进行低功耗休眠，会大概率导致睡死，看门狗重启
        // 或许在蓝牙没有连接成功的时候，应该停止低功耗休眠会好一些。
        // 因此此时的数据会重新产生，但对于下面的设备来说，如果 1 秒内恢复，似乎也没啥。
    }

    // 读取ADC数据并计算平均值
    float readADC() {
        long total = 0;
        for (int i = 0; i < adcSamples; i++) {
            delayMicroseconds(settleTime); // 等待ADC稳定
            total += analogRead(adcPin); // 读取ADC值
        }
        return static_cast<float>(total) / adcSamples; // 计算平均值
    }
};

// 创建ADCController实例，指定相关参数
ADCController adcController(BTN_BUILTIN, 20, 50, 400, 100);

// void onWatchdogOverflow()
// {
//   Serial.println("onWatchdogOverflow");
// }

void setup() {
    Serial.println(__DATE__); // 打印编译日期
    Serial.println(__TIME__); // 打印编译时间
    BLEHandler::getInstance()->begin(); // 初始化BLE服务
    adcController.setup(); // 初始化ADC
    CircularQueue::getInstance()->init(); // 初始化队列
}

void loop() {
    BLEHandler::getInstance()->loop(); // BLE主循环
    adcController.loop(); // ADC主循环
}