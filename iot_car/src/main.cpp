#include <Arduino.h>
#include <ArduinoJson.h>
#include "aliyun_mqtt.h"
#include "PubSubClient.h"
#include "WiFi.h"
#include "Ticker.h"

#define WIFI_SSID "ZPS"         // wifi名
#define WIFI_PASSWD "zps123456" // wifi密码

#define PRODUCT_KEY "gyac4M80uLy"                        //产品ID
#define DEVICE_NAME "iot_car"                            //设备名
#define DEVICE_SECRET "47992ae36c51030543dd8ab4e5dc7fef" //设备key

//设备下发命令的set主题
#define ALINK_TOPIC_PROP_SET "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/service/property/set"
//设备上传数据的post主题
#define ALINK_TOPIC_PROP_POST "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/event/property/post"
//设备post上传数据要用到一个json字符串, 这个是拼接postJson用到的一个字符串
#define ALINK_METHOD_PROP_POST "thing.event.property.post"
//这是post上传数据使用的模板
#define ALINK_BODY_FORMAT "{\"id\":\"%u\",\"version\":\"1.0.1\",\"method\":\"%s\",\"params\":%s}"

int postMsgId = 0; //记录已经post了多少条
Ticker tim1;       //这个定时器是为了每5秒上传一次数据

//上云的标志数据


int front_distance = 300;
//前部超声探测距离0~255
int back_distance = 300;
//后部超声探测距离0~255
int front_warning = 0;
int back_warning = 0;
//枚举类型，告警标志（蜂鸣器响）
// int lightSwitch = 0;
// //布尔类型，小车灯光控制
int night_mode = 0;
//布尔类型，夜间自动检测

struct car_mode
{
  int engine = 0;
  //枚举类型，小车启停及速度
  int move_state = 0;
  //枚举类型，共计十种行动方式
}car_mode1;


//#define LED_out 27
#define LED_in 5
//定义LED灯的引脚
#define light_sensor 21
//定义光电传感器引脚（数字即可）
#define front_distance_out 32
//定义前距离传感器引脚1
#define front_distance_in 34
//定义前距离传感器引脚2
#define back_distance_out 33
//定义前距离传感器引脚1
#define back_distance_in 35
//定义前距离传感器引脚2

//（pwm输出）
//#define front_BEE 23
//#define back_BEE 22
//定义报警装置引脚
#define Motor1_A 16
//定义电机1的正极引脚
#define Motor1_B 17
//定义电机1的负极引脚
#define Motor2_A 15
//定义电机2的正极引脚
#define Motor2_B 14
//定义电机2的负极引脚
#define Motor3_A 18
//定义电机3的正极引脚
#define Motor3_B 19
//定义电机3的负极引脚
#define Motor4_A 13
//定义电机4的正极引脚
#define Motor4_B 12
//定义电机4的负极引脚

//#define test 22
//测试pwm端口

#define Motor1_A_ON ledcWrite(0, 180 * car_mode1.engine)
#define Motor1_A_OFF ledcWrite(0, 0)
#define Motor1_B_ON ledcWrite(1, 180 * car_mode1.engine)
#define Motor1_B_OFF ledcWrite(1, 0)
#define Motor2_A_ON ledcWrite(2, 180 * car_mode1.engine)
#define Motor2_A_OFF ledcWrite(2, 0)
#define Motor2_B_ON ledcWrite(3, 180 * car_mode1.engine)
#define Motor2_B_OFF ledcWrite(3, 0)
#define Motor3_A_ON ledcWrite(4, 180 * car_mode1.engine)
#define Motor3_A_OFF ledcWrite(4, 0)
#define Motor3_B_ON ledcWrite(5, 180 * car_mode1.engine)
#define Motor3_B_OFF ledcWrite(5, 0)
#define Motor4_A_ON ledcWrite(6, 180 * car_mode1.engine)
#define Motor4_A_OFF ledcWrite(6, 0)
#define Motor4_B_ON ledcWrite(7, 180 * car_mode1.engine)
#define Motor4_B_OFF ledcWrite(7, 0)

#define sensor digitalRead(light_sensor)
#define night_on digitalWrite(LED_in, HIGH)
#define night_off digitalWrite(LED_in, LOW)

#define front_warning_on ledcWrite(8,125*front_warning)
#define front_warning_off ledcWrite(8,0)
#define back_warning_on ledcWrite(9,125*back_warning)
#define back_warning_off ledcWrite(9,0)

/*------------------------------------------------------------------------------------------*/

WiFiClient espClient;               //创建网络连接客户端
PubSubClient mqttClient(espClient); //通过网络客户端连接创建mqtt连接客户端

/*------------------------------------------------------------------------------------------*/

//连接WIFI相关函数
void setupWifi()
{
  delay(10);
  Serial.println("连接WIFI");
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (!WiFi.isConnected())
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("OK");
  Serial.println("Wifi连接成功");
}
//重连函数, 如果客户端断线,可以通过此函数重连
void clientReconnect()
{
  while (!mqttClient.connected()) //再重连客户端
  {
    Serial.println("reconnect MQTT...");
    if (connectAliyunMQTT(mqttClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.println("failed");
      Serial.println(mqttClient.state());
      Serial.println("try again in 5 sec");
      delay(5000);
    }
  }
}
void wifiCheck()
{
  if (!WiFi.isConnected()) //先看WIFI是否还在连接
  {
    setupWifi();
  }
  else //如果WIFI连接了,
  {
    if (!mqttClient.connected()) //再看mqtt连接了没
    {
      Serial.println("mqtt disconnected!Try reconnect now...");
      Serial.println(mqttClient.state());
      clientReconnect();
    }
  }
}
// mqtt发布post消息(上传数据)
void mqttPublish()
{
  if (mqttClient.connected())
  {
    //先拼接出json字符串
    char param[192];
    char jsonBuf[768];
    sprintf(param, "{\"Move_state\":%d,\"Engine\":%d,\"Night_mode\":%d,\"Front_distance\":%d,\"Back_distance\":%d,\"Front_Warning\":%d,\"Back_Warning\":%d,\"car_mode\":{\"Engine\":%d,\"Move_state\":%d}}",car_mode1.move_state, car_mode1.engine, sensor, front_distance, back_distance, front_warning,back_warning,car_mode1.engine,car_mode1.move_state); //我们把要上传的数据写在param里
    postMsgId += 1;
    //\"LightSwitch\":%d, digitalRead(LED_out), 
    sprintf(jsonBuf, ALINK_BODY_FORMAT, postMsgId, ALINK_METHOD_PROP_POST, param);
    //再从mqtt客户端中发布post消息
    if (mqttClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf))
    {
      Serial.print("Post message to cloud: ");
      Serial.println(jsonBuf);
      // Serial.println("move_state is:");
      // Serial.println(move_state);
      // Serial.println("engine is:");
      // Serial.println(engine);
      // Serial.println("lightSwitch is:");
      // Serial.println(lightSwitch);
      // Serial.println("front_warning is:");
      // Serial.println(front_warning);
      // Serial.println("back_warning is:");
      // Serial.println(back_warning);
      // Serial.println("front_distance is:");
      // Serial.println(front_distance);
      // Serial.println("back_distance is:");
      // Serial.println(back_distance);
      // Serial.println("light_sensor is:");
      // Serial.println(sensor);
    }
    else
    {
      Serial.println("Publish message to cloud failed!");
    }
  }
}
//收到set主题的命令下发时的回调函数,(接收命令)
void callback(char *topic, byte *payload, unsigned int length)
{
  if (strstr(topic, ALINK_TOPIC_PROP_SET))
  //如果收到的主题里包含字符串ALINK_TOPIC_PROP_SET(也就是收到/sys/a17lGhkKwXs/esp32LightHome/thing/service/property/set主题)
  {
    Serial.println("收到下发的命令主题:");
    Serial.println(topic);
    Serial.println("下发的内容是:");
    payload[length] = '\0'; //为payload添加一个结束附,防止Serial.println()读过了
    Serial.println((char *)payload);

    //接下来是收到的json字符串的解析
    DynamicJsonDocument doc(500);
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
      Serial.println("parse json failed");
      return;
    }
    JsonObject setAlinkMsgObj = doc.as<JsonObject>();
    serializeJsonPretty(setAlinkMsgObj, Serial);
    Serial.println();

    //小车各模块状态更替
    // engine = setAlinkMsgObj["params"]["Engine"];
    // move_state = setAlinkMsgObj["params"]["Move_state"];

    car_mode1.engine = setAlinkMsgObj["params"]["car_mode"]["Engine"];
    car_mode1.move_state = setAlinkMsgObj["params"]["car_mode"]["Move_state"];
    // lightSwitch = setAlinkMsgObj["params"]["LightSwitch"];
    // digitalWrite(LED_out,lightSwitch);
    

    mqttPublish(); //由于将来做应用可能要获取灯的状态,所以在这里发布一下
  }
}

/*------------------------------------------------------------------------------------------*/

//所有小车动作函数
void front_distance_measure()//调试完成
{
  digitalWrite(front_distance_out, LOW);
  delayMicroseconds(5);

  digitalWrite(front_distance_out, HIGH);
  delayMicroseconds(20);
  digitalWrite(front_distance_out, LOW);
  
  front_distance= int(pulseIn(front_distance_in, HIGH)*0.034/2.0);
}
void back_distance_measure()//调试完成
{
  digitalWrite(back_distance_out, LOW);
  delayMicroseconds(5);

  digitalWrite(back_distance_out, HIGH);
  delayMicroseconds(20);
  digitalWrite(back_distance_out, LOW);
  
  back_distance= int(pulseIn(back_distance_in, HIGH)*0.034/2.0);
}
void Car_run()//串口监视器数据调试完毕
{
  if (car_mode1.move_state == 0) //停止
  {
    Motor1_A_OFF;
    Motor1_B_OFF;
    Motor2_A_OFF;
    Motor2_B_OFF;
    Motor3_A_OFF;
    Motor3_B_OFF;
    Motor4_A_OFF;
    Motor4_B_OFF;
  }
  else if (car_mode1.move_state == 1) //前进
  {
    Motor1_A_ON;
    Motor1_B_OFF;
    //左前正转
    Motor2_A_ON;
    Motor2_B_OFF;
    //右前正转
    Motor3_A_ON;
    Motor3_B_OFF;
    //左后正转
    Motor4_A_ON;
    Motor4_B_OFF;
    //右后正转
  }
  else if (car_mode1.move_state == 2) //后退
  {
    Motor1_A_OFF;
    Motor1_B_ON;
    //左前反转
    Motor2_A_OFF;
    Motor2_B_ON;
    //右前反转
    Motor3_A_OFF;
    Motor3_B_ON;
    //左后反转
    Motor4_A_OFF;
    Motor4_B_ON;
    //右后反转
  }
  else if (car_mode1.move_state == 3) //右移
  {
    Motor1_A_OFF;
    Motor1_B_ON;
    //左前反转
    Motor2_A_ON;
    Motor2_B_OFF;
    //右前正转
    Motor3_A_ON;
    Motor3_B_OFF;
    //左后正转
    Motor4_A_OFF;
    Motor4_B_ON;
    //右后反转
  }
  else if (car_mode1.move_state == 4) //左移
  {
    Motor1_A_ON;
    Motor1_B_OFF;
    //左前正转
    Motor2_A_OFF;
    Motor2_B_ON;
    //右前反转
    Motor3_A_OFF;
    Motor3_B_ON;
    //左后反转
    Motor4_A_ON;
    Motor4_B_OFF;
    //右后正转
  }
  else if (car_mode1.move_state == 5) //左旋转
  {
    Motor1_A_OFF;
    Motor1_B_ON;
    //左前反转
    Motor2_A_ON;
    Motor2_B_OFF;
    //右前正转
    Motor3_A_OFF;
    Motor3_B_ON;
    //左后反转
    Motor4_A_ON;
    Motor4_B_OFF;
    //右后正转
  }
  else //右旋转
  {
    Motor1_A_ON;
    Motor1_B_OFF;
    //左前正转
    Motor2_A_OFF;
    Motor2_B_ON;
    //右前反转
    Motor3_A_ON;
    Motor3_B_OFF;
    //左后正转
    Motor4_A_OFF;
    Motor4_B_ON;
    //右后反转  
  }
}
void night_check()//调试完成
{
  if (sensor == 1) //光线不足
  {
    night_on;
    night_mode = 1;
  }
  else //光线充足
  {
    night_off;
    night_mode = 0;
  }
}
void Front_Warn()//串口监视器数据调试完毕
{
  front_distance_measure();
  if(front_distance>=25)
  {
    front_warning = 0;
    front_warning_off;
  }
  else if(front_distance<25&&front_distance>=20)
  {
    front_warning = 1;
    front_warning_on;
  }
  else if(front_distance<20&&front_distance>=15)
  {
    front_warning = 2;
    front_warning_on;
  }
  else if(front_distance<15&&front_distance>=10)
  {
    front_warning = 3;
    front_warning_on;
  }
  else if(front_distance<10)
  {
    front_warning = 4;
    front_warning_on;
  }
}
void Back_Warn()//串口监视器数据调试完毕
{
  back_distance_measure();
  if(back_distance>=25)
  {
    back_warning = 0;
    back_warning_off;
  }
  else if(back_distance<25&&back_distance>=20)
  {
    back_warning = 1;
    back_warning_on;
  }
  else if(back_distance<20&&back_distance>=15)
  {
    back_warning = 2;
    back_warning_on;
  }
  else if(back_distance<15&&back_distance>=10)
  {
    back_warning = 3;
    back_warning_on;
  }
  else if(back_distance<10)
  {
    back_warning = 4;
    back_warning_on;
  }
}

/*------------------------------------------------------------------------------------------*/

void setup()
{
  //通用GPIO端口
  pinMode(LED_in, OUTPUT);
  //pinMode(LED_out,OUTPUT);
  pinMode(light_sensor, INPUT);
  pinMode(front_distance_in, INPUT);
  pinMode(front_distance_out, OUTPUT);
  pinMode(back_distance_in, INPUT);
  pinMode(back_distance_out, OUTPUT);

  //所有pwm输出端口
  //pinMode(front_BEE, OUTPUT);
  //pinMode(back_BEE, OUTPUT);
  pinMode(Motor1_A, OUTPUT);
  pinMode(Motor1_B, OUTPUT);
  pinMode(Motor2_A, OUTPUT);
  pinMode(Motor2_B, OUTPUT);
  pinMode(Motor3_A, OUTPUT);
  pinMode(Motor3_B, OUTPUT);
  pinMode(Motor4_A, OUTPUT);
  pinMode(Motor4_B, OUTPUT);

  //设置LEDC通道0~8频率为256，分辨率为10位，即占空比可选0~1023将，所有的通道对应
  ledcSetup(8, 256, 10);
  ledcSetup(9, 256, 10);
  ledcSetup(0, 256, 10);
  ledcSetup(1, 256, 10);
  ledcSetup(2, 256, 10);
  ledcSetup(3, 256, 10);
  ledcSetup(4, 256, 10);
  ledcSetup(5, 256, 10);
  ledcSetup(6, 256, 10);
  ledcSetup(7, 256, 10);
  //ledcAttachPin(front_BEE, 8);
  //ledcAttachPin(back_BEE, 9);cx发v   
  ledcAttachPin(Motor1_A, 0);
  ledcAttachPin(Motor1_B, 1);
  ledcAttachPin(Motor2_A, 2);
  ledcAttachPin(Motor2_B, 3);
  ledcAttachPin(Motor3_A, 4);
  ledcAttachPin(Motor3_B, 5);
  ledcAttachPin(Motor4_A, 6);
  ledcAttachPin(Motor4_B, 7);

  //测试用端口
  pinMode(12, INPUT);

  Serial.begin(115200);
  setupWifi();
  if (connectAliyunMQTT(mqttClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET))
  {
    Serial.println("MQTT服务器连接成功!");
  };
  mqttClient.setCallback(callback); //绑定收到set主题时的回调(命令下发回调)
  tim1.attach(5, mqttPublish);      //启动每5秒发布一次消息
}

void loop()
{
  //检测有没有断线
  wifiCheck();
  // mqtt客户端监听
  mqttClient.loop();

  //小车运动函数
  Car_run();
  //小车内部仪表背景灯开关函数
  night_check();
  //蜂鸣器报警函数
  Front_Warn();
  Back_Warn();

  //pwm探测调试端口
  //Serial.println(digitalRead(12));
}