#include <Arduino.h>

// 定义数据结构
struct data {
    uint32_t time;
    float source;
};

// 循环队列类
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
            Serial.println("Invalid input data.");
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

    // 打印队列中的所有数据
    void printQueue() {
        Serial.println("Queue contents:");
        int current = head;
        for (int i = 0; i < count; i++) {
            Serial.print("time: ");
            Serial.print(queue[current].time);
            Serial.print(", source: ");
            Serial.println(queue[current].source);
            current = (current + 1) % MAX_SIZE;
        }
    }
};

// 初始化单例实例
CircularQueue CircularQueue::instance;

void setup() {
    Serial.begin(115200); // 初始化串口通信
    CircularQueue* queue = CircularQueue::getInstance();

    // 显式调用初始化函数
    queue->init();

    // 模拟入队操作，插入 200 个元素
    Serial.println("Inserting 200 elements...");
    for (int i = 0; i < 200; i++) {
        data d{i * 1000, static_cast<float>(i) * 0.1f};
        queue->enqueue(&d);
    }

    // 打印队列
    Serial.println("Queue after inserting 200 elements:");
    queue->printQueue();

    // 模拟出队操作，删除 200 个元素
    Serial.println("Deleting 200 elements...");
    data item;
    for (int i = 0; i < 200; i++) {
        if (queue->dequeue(&item)) {
            Serial.print("Dequeued: time = ");
            Serial.print(item.time);
            Serial.print(", source = ");
            Serial.println(item.source);
        } else {
            Serial.println("Queue is empty.");
            break;
        }
    }

    // 打印队列
    Serial.println("Queue after deleting 200 elements:");
    queue->printQueue();

    // 重置队列
    queue->reset();
    Serial.println("Queue has been reset.");
}

void loop() {
    // 主循环为空
}