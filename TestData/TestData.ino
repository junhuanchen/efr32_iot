#include <Arduino.h>
// 数据头定义
const uint8_t DATA_HEADER = 0xA5;

// 数据包结构
struct DataPacket {
    uint8_t header;       // 数据头
    uint8_t length;       // 数据长度
    uint8_t data[60];     // 数据内容（最大长度为60字节）
    uint8_t checksum;     // 校验和
};

// 状态枚举
enum State {
    WAIT_FOR_HEADER,
    READ_LENGTH,
    READ_DATA,
    CHECKSUM
};

// 定义 RingBuffer 的大小
static const size_t data_buffer_size = 64*1024u; // xg24explorerkit
// static const size_t data_buffer_size = 2*1024u; // bgm220

struct SerialProtocol {
    // 发送数据
    bool sendData(const uint8_t* data, size_t length) {
        if (length > 60) {
            return false; // 数据长度超出限制
        }

        DataPacket packet;
        packet.header = DATA_HEADER;
        packet.length = static_cast<uint8_t>(length);
        memcpy(packet.data, data, length);

        // 计算校验和
        uint8_t checksum = DATA_HEADER + packet.length;
        for (size_t i = 0; i < length; ++i) {
            checksum += packet.data[i];
        }
        packet.checksum = checksum;

        // 构造数据包
        Serial1.write(packet.header);
        Serial1.write(packet.length);
        Serial1.write(packet.data, length);
        Serial1.write(packet.checksum);

        return true;
    }

    // 接收数据
    bool receiveData(uint8_t* data, size_t& length) {
        // Serial.printf("Serial1.available() %d \r\n", Serial1.available());
        if (Serial1.available() > 0) {
            // 检查超时
            if (millis() - _lastActivityTime > _timeoutMs) {
                _lastActivityTime = millis();
                // 超时重置
                reset();
                return false;
            }

            uint8_t byte = Serial1.read();
            _lastActivityTime = millis(); // 更新最后活动时间
            // Serial.printf("byte %d _state %d \r\n", byte, _state);

            switch (_state) {
                case WAIT_FOR_HEADER:
                    if (byte == DATA_HEADER) {
                        _state = READ_LENGTH;
                    }
                    break;
                case READ_LENGTH:
                    _length = byte;
                    if (_length > 60) {
                        // 数据长度不合理，重置状态机
                        reset();
                        return false;
                    }
                    _state = READ_DATA;
                    break;
                case READ_DATA:
                    if (!_data.isFull()) {
                        _data.store_char(byte);
                    }
                    if (_data.available() == _length) {
                        _state = CHECKSUM;
                    }
                    break;
                case CHECKSUM:
                    // 校验数据
                    uint8_t calculatedChecksum = DATA_HEADER + _length;
                    size_t i = 0;
                    while (_data.available() > 0) {
                        uint8_t tmp = _data.read_char();
                        calculatedChecksum += tmp;
                        data[i++] = tmp;
                    }
                    // Serial.printf("byte %d calculatedChecksum %d \r\n", byte, calculatedChecksum);
                    if (calculatedChecksum == byte) {
                        // 校验成功
                        length = _length;
                        reset();
                        // Serial.println("have data");
                        return true;
                    } else {
                        // 校验失败，重置状态机
                        reset();
                        return false;
                    }
            }
        }
        return false;
    }

    void reset() {
        _state = WAIT_FOR_HEADER;
        _length = 0;
        _data.clear();
    }

    void init(unsigned long timeoutMs = 1000) {
        _state = WAIT_FOR_HEADER;
        _timeoutMs = timeoutMs;
        _lastActivityTime = millis();
        // memset(_data._aucBuffer, 0, data_buffer_size);
        _data.clear();
        Serial1.begin(115200);
    }

    State _state;
    uint8_t _length;
    RingBufferN<data_buffer_size> _data; // 使用 RingBufferN 替代 std::queue
    unsigned long _timeoutMs; // 超时时间（毫秒）
    unsigned long _lastActivityTime; // 最后一次活动时间
};

SerialProtocol protocol;

void setup() {
    protocol.init(); // 初始化 SerialProtocol
    Serial.begin(115200);
    Serial.println("Serial Protocol Example");
}

void loop() {

    static uint8_t dataToSend[] = {1, 2, 3, 4, 5};
    static uint8_t dataReceived[60];
    static size_t receivedLength;

    // Serial.printf("lastSendTime %d\r\n", millis());

    // 每隔一段时间发送数据
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime > 2000) { // 每2秒发送一次数据
        if (protocol.sendData(dataToSend, sizeof(dataToSend))) {
            Serial.println("Data sent successfully.");
        } else {
            Serial.println("Failed to send data.");
        }
        lastSendTime = millis();
    }

    // 尝试接收数据
    if (protocol.receiveData(dataReceived, receivedLength)) {
        Serial.print("Received valid packet, length: ");
        Serial.println(receivedLength);
        Serial.print("Data: ");
        for (size_t i = 0; i < receivedLength; ++i) {
            Serial.print(dataReceived[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
}
