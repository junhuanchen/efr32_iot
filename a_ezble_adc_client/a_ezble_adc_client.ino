#include <ezBLE.h>
#include <ArduinoLowPower.h>
#include <EEPROM.h>
#include <WatchdogTimer.h> // 引入看门狗库

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
  // static const size_t data_buffer_size = 128*1024u;
  // RingBufferN<data_buffer_size> data;

  // 私有构造函数，防止外部直接创建实例
  BLEHandler() {
    Serial.begin(115200);
    Serial.println("ezBLE dls client");
    Serial.println(__DATE__); // 打印编译日期
    Serial.println(__TIME__); // 打印编译时间
    // Serial.printf("EEPROM.length() %d\r\n", EEPROM.length());
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

    // 启动看门狗定时器，设置超时时间为2秒
    WatchdogTimer.begin(WDOG_PERIOD_2_S);

    Serial.println("Waiting for BLE connection...");
  }

  // 处理接收到的数据
  static void onReceive(int bytes) {
    (void)bytes; // 忽略bytes参数
    BLEHandler* handler = BLEHandler::getInstance(); // 获取单例实例

    while (ezBLE.available()) {
      char c = (char)ezBLE.read();
      Serial.print(c);

      if (c == '\n') { // 如果接收到换行符
        // 处理接收到的完整字符串
        if (handler->receivedData.length() > 0) {
          // 假设接收到的字符串直接作为新的服务名称
          handler->receivedData.toCharArray(bleConfig.serviceName, sizeof(bleConfig.serviceName));
          // 重启设备
          handler->restart();
        }
        handler->receivedData = ""; // 清空接收缓冲区
      } else {
        handler->receivedData += c; // 将字符追加到接收缓冲区
      }
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
    if (Serial.available()) {
      while (Serial.available())
      {
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
  }

  // 主循环
  void loop() {
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

    // char buffer[128];
    // sprintf(buffer, "%d 0123456789abcdefghijklmnopqrstuvwsyz0123456789abcdefghijklmnopqrstuvwsyz\n", millis());
    // send(buffer); // 发送数据
    // Serial.printf("%d Sending data over ezBLE\n", millis());
    // delay(100);
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

      // 延迟指定时间
      delayMicroseconds(delay_time);

      // 采集ADC数据并计算平均值
      float average = readADC();

      // 恢复Inamp和桥驱动的初始状态
      digitalWrite(4, HIGH); // Inamp enable
      digitalWrite(14, LOW); // Inamp reference buffer
      digitalWrite(13, HIGH);  // Drive bridge
      digitalWrite(9, LOW); // Turn off ADC driver buffer

      // 打印采集到的平均值
      Serial.print("Time for ");
      Serial.print(delay_time);
      Serial.print("us: ");
      Serial.println(average);

      // 创建一个String对象并拼接数据
      String message = "ADC Average: ";
      message.concat(average);

      // 通过ezBLE发送数据
      BLEHandler::getInstance()->send(message.c_str());

      // 低功耗睡眠模式
      LowPower.sleep(98);
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
ADCController adcController(PA4, 20, 50, 400, 100);

void setup() {
    Serial.println(__DATE__); // 打印编译日期
    Serial.println(__TIME__); // 打印编译时间
    BLEHandler::getInstance()->begin(); // 初始化BLE服务
    adcController.setup(); // 初始化ADC
}

void loop() {

    BLEHandler::getInstance()->loop(); // BLE主循环
    // 如果设备未连接，不执行后续操作
    if (!BLEHandler::getInstance()->isConnected) {
      return;
    }
    
    adcController.loop(); // ADC主循环
}
