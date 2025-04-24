#include <ezBLE.h>
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

  // 私有构造函数，防止外部直接创建实例
  BLEHandler() {
    Serial.begin(115200); // 初始化主串口
    Serial1.begin(115200); // 初始化串口1
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

  // 处理串口1输入
  void handleSerial1Input() {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') { // 如果接收到换行符
        // 将串口1接收到的完整字符串发送到BLE
        if (serialData.length() > 0) {
          send(serialData.c_str());
          Serial.printf("Forwarding data to BLE: %s\n", serialData.c_str());
          serialData = ""; // 清空串口接收缓冲区
        }
      } else {
        serialData += c; // 将字符追加到串口接收缓冲区
      }
    }
  }

  // 主循环
  void loop() {
    // 喂狗，防止看门狗触发重启
    WatchdogTimer.feed();

    // 处理串口输入
    handleSerialInput();

    // 处理串口1输入
    handleSerial1Input();

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

void setup() {
  BLEHandler::getInstance()->begin(); // 初始化BLE服务
}

void loop() {
  BLEHandler::getInstance()->loop(); // 主循环
}