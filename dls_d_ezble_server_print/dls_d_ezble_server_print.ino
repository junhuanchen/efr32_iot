#include <ezBLE.h>
#include <EEPROM.h>
#include <ArduinoLowPower.h>
#include <WatchdogTimer.h> // 引入看门狗库

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
struct CircularQueue {
  
    static CircularQueue instance; // 静态单例实例
    static const int MAX_SIZE = 100; // 队列最大容量
    data queue[MAX_SIZE]; // 存储数据的静态数组
    int head; // 队列头部指针
    int tail; // 队列尾部指针
    int count; // 队列中元素的数量

    CircularQueue() { head = tail = count = 0; } // 私有构造函数，仅用于声明

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
            Serial.println("Invalid input data.");
            return;
        }
        if (count == MAX_SIZE) {
            // 队列满了，自动出队清理
            // Serial.println("Queue is full, dequeueing to make space.");
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
    uint8_t buffer[64];           // 临时缓冲区
    int bufferIndex;              // 缓冲区索引
    bool isReceiving;             // 是否正在接收数据包

    // 私有构造函数，防止外部直接创建实例
    BLEHandler() {
        Serial.begin(115200); // 初始化串口
        Serial1.begin(115200); // 初始化Serial1
        Serial.println("ezBLE dls server");
        Serial.println(__DATE__); // 打印编译日期
        Serial.println(__TIME__); // 打印编译时间

        // 从EEPROM读取配置
        EEPROM.get(0, bleConfig);
        if (strlen(bleConfig.serviceName) == 0) {
            strcpy(bleConfig.serviceName, "defaultService");
        }
        
        Serial.printf("Service name: %s\n", bleConfig.serviceName);

        isConnected = false; // 初始状态为未连接
        bufferIndex = 0;
        isReceiving = false;
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
        ezBLE.beginServer(bleConfig.serviceName);
        ezBLE.onReceive(onReceive); // 注册接收回调

        // 启动看门狗定时器，设置超时时间为2秒
        WatchdogTimer.begin(WDOG_PERIOD_2_S);

        Serial.println("Waiting for BLE connection...");
    }

    // 处理接收到的数据
    static void onReceive(int bytes) {
        (void)bytes; // 忽略bytes参数
        BLEHandler* handler = BLEHandler::getInstance(); // 获取单例实例

        while (ezBLE.available()) {
            uint8_t c = ezBLE.read();
            // Serial.print(c); // 在串口监视器中打印接收到的数据
            // Serial1.print(c); // 将接收到的数据通过Serial1输出

            if (!handler->isReceiving) {
                if (c == 0xA5) { // 数据头
                    handler->isReceiving = true;
                    handler->bufferIndex = 0;
                    handler->buffer[handler->bufferIndex++] = c;
                }
            } else {
                handler->buffer[handler->bufferIndex++] = c;
                if (handler->bufferIndex == 4) { // 已经读取了头部和长度
                    uint8_t length = handler->buffer[1];
                    if (handler->bufferIndex + length > sizeof(handler->buffer)) {
                        Serial.println("Error: Buffer overflow.");
                        handler->isReceiving = false;
                        return;
                    }
                }
                if (handler->bufferIndex == 2 + handler->buffer[1] + 1) { // 数据和校验和接收完成
                    handler->isReceiving = false;
                    if (handler->validateChecksum()) {
                        handler->processDataPacket();
                    } else {
                        Serial.println("Error: Checksum mismatch.");
                    }
                }
            }
        }
    }

    // 校验数据包的校验和
    bool validateChecksum() {
        uint8_t checksum = 0;
        for (int i = 0; i < bufferIndex - 1; i++) {
            checksum += buffer[i];
        }
        return checksum == buffer[bufferIndex - 1];
    }

    // 处理数据包
    void processDataPacket() {
        DataPacket* packet = reinterpret_cast<DataPacket*>(buffer);
        if (packet->length == sizeof(data)) {
            data adcData;
            memcpy(&adcData.adc_source, packet->data, sizeof(data));
            CircularQueue::getInstance()->enqueue(&adcData);
            Serial.print("Received and enqueued data: ");
            Serial.println(adcData.adc_source);
        } else {
            Serial.println("Error: Invalid data length.");
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
        ezBLE.printf("%s\n", data);
    }

    // 处理串口输入
    void handleSerialInput() {
        if (Serial.available()) {
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
    }

    // 主循环
    void loop() {
        
        // LowPower.sleep(20);

        // 喂狗，防止看门狗触发重启
        WatchdogTimer.feed();

        // 处理串口输入
        handleSerialInput();

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

        // 从队列中获取数据并打印
        if (!CircularQueue::getInstance()->isEmpty()) {
            data adcData;
            if (CircularQueue::getInstance()->dequeue(&adcData)) {
                Serial.print("Dequeued count: ");
                Serial.print(CircularQueue::getInstance()->count);
                Serial.print(" data: ");
                Serial.println(adcData.adc_source);
            }
        }
    }
};

// 初始化单例实例指针
BLEHandler* BLEHandler::instance = nullptr;

void setup() {
    BLEHandler::getInstance()->begin(); // 初始化BLE服务
    CircularQueue::getInstance()->init(); // 初始化队列
}

void loop() {
    BLEHandler::getInstance()->loop(); // 主循环
}