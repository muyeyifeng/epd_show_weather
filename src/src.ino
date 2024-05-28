//    Copyright [2024] [muyeyifeng@gmail.com]

//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at

//        http://www.apache.org/licenses/LICENSE-2.0

//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.
/*******************字符点阵定义***********************/
#include "src.h"
//#include "NetworkTime.h"

/**********************全局变量**********************/
Ticker clockTimer;

// 创建TimeClient实例
TimeClient_CTM networkTime(timeApiUrl);

bool flag = true;
bool isBusy = false;

unsigned char clear[33 * 2];
unsigned char clearWeather[42 * 56 / 8];
unsigned char* volteLable;
unsigned char* voltePerLable;
unsigned char* ntTimeLable;

static unsigned long lastWeatherSync = 0;

/*********************setup*********************/
void setup(void) {
  // 初始化USB串口通讯
  Serial.begin(115200);
  delay(500);

  // 打印初始信息
  Serial.println();
  Serial.println("Booting Sketch...");

  // 初始化SPI端口
  pinMode(BUSY_Pin, INPUT);
  pinMode(RES_Pin, OUTPUT);
  pinMode(DC_Pin, OUTPUT);
  pinMode(CS_Pin, OUTPUT);
  pinMode(SCK_Pin, OUTPUT);
  pinMode(SDI_Pin, OUTPUT);

  // 连接wifi
  // delete old config
  WiFi.config(ip_local, geta_way, subnet, dns1, dns2);
  WiFi.disconnect(true);
  //WiFi.mode(WIFI_STA);
  //scanwifi();
  delay(500);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
      delay(500);
      Serial.print(".");
      retries++;
    }
  }
  checkWifiStatus();

  // 初始化信号灯
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // 初始化电池检测
  pinMode(vccPin, INPUT);
  pinMode(vddPin, OUTPUT);

  // 设置Vdd输出为0作为GND
  digitalWrite(vddPin, LOW);

  EPD_HW_Init();  // Electronic paper initialization
  EPD_SetRAMValue_BaseMap(gImage_frame);
  EPD_DeepSleep();  // Enter deep sleep,Sleep instruction is necessary, please do not delete!!!

  memset(clear, 0xFF, sizeof(clear));
  memset(clearWeather, 0xFF, sizeof(clearWeather));

  // 初次更新数据
  networkTime.updateTime();
  updateWeather();
  updateTime();
  updateBattery();
  EPD_Part_Update();

  // 设置定时任务
  clockTimer.attach(1, updateSysInfo);
}

/*********************loop*********************/
void loop(void) {
  static unsigned long lastSync = 0;
  if (millis() - lastSync > 3600000 && isWifiConnected()) {  // 3600000 毫秒 = 1 小时
    networkTime.updateTime();
    lastSync = millis();
  }
  delay(1000);
}

/****************************自定义函数实现************************************/
// 更新当前电压
void updateBattery() {
  const int battery = analogRead(vccPin);
  const int inter = (int)(battery * 3.3 / 4095);
  const int fact = (int)(battery * 10 * 3.3 / 4095) % 10;
  const String volte = inter + String('.') + fact + String('V');  //3.3V
  const String voltePer = String(battery).substring(1);           //  4(095)
  int volteLength = 0;
  int voltePerLength = 0;

  volteLable = extractUtf8Characters(volte, &volteLength);
  voltePerLable = extractUtf8Characters(voltePer, &voltePerLength);

  EPD_Dis_Part(40 - 16, 296, clear, 33, 16);
  EPD_Dis_Part(40 - 16, 296 - 16 + volteLength * 4, volteLable, volteLength * 8, 16);

  EPD_Dis_Part(40, 296, clear, 33, 16);
  EPD_Dis_Part(40, 296 - 16 + voltePerLength * 4, voltePerLable, voltePerLength * 8, 16);
}

// 更新当前时间
void updateTime(int xaddr, int yaddr) {
  if (WiFi.status() == WL_CONNECTED) {
    String ntTime = networkTime.getCurrentTime();
    String ntDate = networkTime.getCurrentDate();
    String ntWeek = networkTime.getDayOfWeek();

    //Serial.println(ntTime);
    String timeString = ntDate + " 周" + ntWeek + " " + ntTime;
    int length = 0;
    ntTimeLable = extractUtf8Characters(timeString, &length);
    EPD_Dis_Part(128 - xaddr, 296 - yaddr, ntTimeLable, length * 8, 16);
  } else {
    networkTime.updateTime();
  }
}

// 更新天气预报
void updateWeather() {
  JsonDocument weatherJson;
  if (isWifiConnected()) {
    int httpCode = getWeatherJson(&weatherJson);
    if (httpCode == 200) {
      if (Serial) {
        serializeJson(weatherJson, Serial);
        Serial.println("");
      }
      //Serial.println((int)weatherJson["status"]);
    } else {
      if (Serial) Serial.println("Http requests fail......");
      return;
    }
  } else {
    scanwifi();
    WiFi.disconnect(true);
    WiFi.begin(ssid, password);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
      delay(500);
      if (Serial) Serial.print(".");
      retries++;
    }
    checkWifiStatus();
    return;
  }

  updateTime(32);
  if (weatherJson != NULL && (int)weatherJson["status"] == 0) {
    int i;
    const char* weatherNowText = weatherJson["result"]["now"]["text"];
    if (Serial) Serial.println(weatherNowText);
    const unsigned char* weatherNow = findWeather(weatherNowText);

    int weatherNowTemp = weatherJson["result"]["now"]["temp"];
    int length;
    unsigned char* nowLable = extractUtf8Characters(String(weatherNowTemp) + String("℃"), &length);

    unsigned char* resizeWeatherNow;
    resizeImage(weatherNow, 40, 40, &resizeWeatherNow, 48, 48);
    // for(int k = 0; k < 40 * 40 / 8; k++){
    //   Serial.print(resizeWeatherNow[k], HEX);
    // }
    int length2;
    unsigned char* resizeLable = extractUtf8Characters(String(weatherNowText) + " " + String(weatherNowTemp) + String("℃"), &length2);

    JsonArray values = weatherJson["result"]["forecasts"];
    for (i = 0; i < 2; i++) {
      //更新天气图标前将当日位置点阵置为白
      EPD_Dis_Part(0, 296 - 36, clearWeather, 42, 56);         //上排小图标重置
      EPD_Dis_Part(128 - 72, 296, gImage_leftbottom, 77, 72);  //下排大图标重置

      EPD_Dis_Part(128 - 64, 296 - 14, resizeWeatherNow, 48, 48);                         //下排大图标更新
      EPD_Dis_Part(128 - 16, 296 - 14 - 25 + length2 * 4, resizeLable, length2 * 8, 16);  //下排大图标文字更新

      EPD_Dis_Part(40, 296 - 37 - 20 + length * 4, nowLable, length * 8, 16);  //上排小图标文字更新
      EPD_Dis_Part(0, 296 - 37, weatherNow, 40, 40);                           //上排小图标更新
      //Serial.println(values);
      int start = 81;
      int step = 44;
      int j = 0;
      for (JsonObject obj : values) {
        //更新天气图标前将当前位置点阵置为白
        EPD_Dis_Part(0, 296 - start + 1 - j * step, clearWeather, 42, 56);

        const char* text_day = obj["text_day"];
        const unsigned char* weatherForecasts = findWeather(text_day);
        EPD_Dis_Part(0, 296 - start - j * step, weatherForecasts, 40, 40);

        int high = obj["high"];
        int low = obj["low"];
        String label = low + String("-") + high;
        int length1;
        unsigned char* tempLabel = extractUtf8Characters(label, &length1);
        unsigned char clear[84];

        EPD_Dis_Part(40, 296 - start - j * step - 20 + length1 * 4, tempLabel, length1 * 8, 16);
        j++;
      }
      EPD_Part_Update();
    }
  }
}

// 系统信息定时更新
// 所有局刷的系统信息统一更新后再刷新
void updateSysInfo() {
  if (millis() - lastWeatherSync > 3600000 / 2) {  // 3600000 毫秒 = 1 小时
    updateWeather();
    lastWeatherSync = millis();
  }
  updateTime();
  updateBattery();
  EPD_Part_Update();
}
// 扫描wifi列表
void scanwifi() {
  // 扫描WiFi网络
  int numNetworks = WiFi.scanNetworks();

  if (numNetworks == 0) {
    if (Serial) Serial.println("No WiFi networks found");
  } else {
    if (Serial) {
      Serial.print("Found ");
      Serial.print(numNetworks);
      Serial.println(" WiFi networks:");
    }
    for (int i = 0; i < numNetworks; i++) {
      if (Serial) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));  // 获取SSID
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));  // 获取信号强度
        Serial.println(")");
      }
    }
  }
}

// 尝试连接wifi
void checkWifiStatus() {
  switch (WiFi.status()) {
    case WL_NO_SSID_AVAIL:
      if (Serial) Serial.println("[WiFi] SSID not found");
      break;
    case WL_CONNECT_FAILED:
      if (Serial) Serial.print("[WiFi] Failed - WiFi not connected! Reason: ");
      return;
      break;
    case WL_CONNECTION_LOST:
      if (Serial) Serial.println("[WiFi] Connection was lost");
      break;
    case WL_SCAN_COMPLETED:
      if (Serial) Serial.println("[WiFi] Scan is completed");
      break;
    case WL_DISCONNECTED:
      if (Serial) Serial.println("[WiFi] WiFi is disconnected");
      break;
    case WL_CONNECTED:
      if (Serial) {
        Serial.println("[WiFi] WiFi is connected!");
        Serial.print("[WiFi] IP address: ");
        Serial.println(WiFi.localIP());
      }
      return;
      break;
    default:
      if (Serial) Serial.print("[WiFi] WiFi Status: ");
      if (Serial) Serial.println(WiFi.status());
      break;
  }
}

// 返回连接状态
bool isWifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

/////////////////////delay//////////////////////////////////////
void driver_delay_us(unsigned int xus) {  // 1us
  for (; xus > 1; xus--)
    ;
}

void driver_delay_xms(unsigned long xms) {  // 1ms
  unsigned long i = 0, j = 0;
  for (j = 0; j < xms; j++) {
    for (i = 0; i < 256; i++)
      ;
  }
}

void DELAY_S(unsigned int delaytime) {
  int i, j, k;
  for (i = 0; i < delaytime; i++) {
    for (j = 0; j < 4000; j++) {
      for (k = 0; k < 222; k++)
        ;
    }
  }
}

void SPI_Write(unsigned char TxData) {
  unsigned char i;
  unsigned char value;
  value = TxData;
  EPD_W21_CLK_0;
  for (i = 0; i < 8; i++) {
    if (value & 0x80)
      EPD_W21_MOSI_1;
    else
      EPD_W21_MOSI_0;
    EPD_W21_CLK_1;
    EPD_W21_CLK_0;
    value = (value << 1);
  }
}

void SPI_Delay(unsigned char xrate) {
  unsigned char i;
  while (xrate) {
    for (i = 0; i < 2; i++)
      ;
    xrate--;
  }
}

/////////////////EPD Write/////////////////////
void Epaper_Write_Command(unsigned char command) {
  EPD_W21_DC_0;  // command write
  EPD_W21_CS_0;
  SPI_Write(command);
  EPD_W21_CS_1;
}

void Epaper_Write_Data(unsigned char datas) {
  EPD_W21_DC_1;  // data write
  EPD_W21_CS_0;
  SPI_Write(datas);
  EPD_W21_CS_1;
}

/////////////////EPD settings Functions/////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
void EPD_HW_Init(void) {
  EPD_W21_RST_1;
  delay(200);     // At least 10ms delay
  EPD_W21_RST_0;  // Module reset
  delay(200);     // At least 10ms delay
  EPD_W21_RST_1;
  delay(200);  // At least 10ms delay

  Epaper_READBUSY();
  Epaper_Write_Command(0x12);  // SWRESET
  Epaper_READBUSY();

  Epaper_Write_Command(0x01);  // Driver output control
  Epaper_Write_Data(0x27);
  Epaper_Write_Data(0x01);
  Epaper_Write_Data(0x00);

  Epaper_Write_Command(0x11);  // data entry mode
  Epaper_Write_Data(0x01);

  Epaper_Write_Command(0x44);  // set Ram-X address start/end position
  Epaper_Write_Data(0x00);
  Epaper_Write_Data(0x0F);  // 0x0F-->(15+1)*8=128

  Epaper_Write_Command(0x45);  // set Ram-Y address start/end position
  Epaper_Write_Data(0x27);     // 0x0127-->(295+1)=296
  Epaper_Write_Data(0x01);
  Epaper_Write_Data(0x00);
  Epaper_Write_Data(0x00);

  Epaper_Write_Command(0x3C);  // BorderWavefrom
  Epaper_Write_Data(0x05);

  Epaper_Write_Command(0x18);  // Read built-in temperature sensor
  Epaper_Write_Data(0x80);

  Epaper_Write_Command(0x21);  //  Display update control
  Epaper_Write_Data(0x00);
  Epaper_Write_Data(0x80);

  Epaper_Write_Command(0x4E);  // set RAM x address count to 0;
  Epaper_Write_Data(0x00);
  Epaper_Write_Command(0x4F);  // set RAM y address count to 0X199;
  Epaper_Write_Data(0x27);
  Epaper_Write_Data(0x01);
  Epaper_READBUSY();
}
//////////////////////////////All screen update////////////////////////////////////////////
void EPD_WriteScreen_ALL(const unsigned char* datas) {
  unsigned int i;
  Epaper_Write_Command(0x24);  // write RAM for black(0)/white (1)
  int size = ALLSCREEN_GRAGHBYTES;
  for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
    if (i < size)
      Epaper_Write_Data(pgm_read_byte(&datas[i]));
    else
      Epaper_Write_Data(0x00);
  }
  EPD_Update();
}

/////////////////////////////////////////////////////////////////////////////////////////
void EPD_Update(void) {
  Epaper_Write_Command(0x22);  // Display Update Control
  Epaper_Write_Data(0xF7);
  Epaper_Write_Command(0x20);  // Activate Display Update Sequence
  Epaper_READBUSY();
}

void EPD_Part_Update(void) {
  Epaper_Write_Command(0x22);  // Display Update Control
  Epaper_Write_Data(0xFF);
  Epaper_Write_Command(0x20);  // Activate Display Update Sequence
  Epaper_READBUSY();
}

void EPD_DeepSleep(void) {
  Epaper_Write_Command(0x10);  // enter deep sleep
  Epaper_Write_Data(0x01);
  delay(100);
}
void Epaper_READBUSY(void) {
  while (1) {  //=1 BUSY
    if (isEPD_W21_BUSY == 0)
      break;
  }
}
///////////////////////////Part update//////////////////////////////////////////////
void EPD_SetRAMValue_BaseMap(const unsigned char* datas) {
  unsigned int i;
  const unsigned char* datas_flag;
  datas_flag = datas;
  Epaper_Write_Command(0x24);  // Write Black and White image to RAM
  for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
    Epaper_Write_Data(pgm_read_byte(&datas[i]));
  }
  datas = datas_flag;
  Epaper_Write_Command(0x26);  // Write Black and White image to RAM
  for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
    Epaper_Write_Data(pgm_read_byte(&datas[i]));
  }
  EPD_Update();  //?
}

void EPD_Dis_Part(unsigned int x_start, unsigned int y_start, const unsigned char* datas, unsigned int PART_COLUMN, unsigned int PART_LINE) {
  unsigned int i;
  unsigned int x_end, y_start1, y_start2, y_end1, y_end2;
  x_start = x_start / 8;  //
  x_end = x_start + PART_LINE / 8 - 1;

  y_start1 = 0;
  y_start2 = y_start;

  if (y_start >= 256) {
    y_start1 = y_start2 / 256;
    y_start2 = y_start2 % 256;
  }
  y_end1 = 0;
  y_end2 = y_start - PART_COLUMN;

  if (y_end2 >= 256) {
    y_end1 = y_end2 / 256;
    y_end2 = y_end2 % 256;
  }
  // Reset
  EPD_W21_RST_0;  // Module reset
  delay(10);      // At least 10ms delay
  EPD_W21_RST_1;
  delay(10);  // At least 10ms delay

  Epaper_Write_Command(0x3C);  // BorderWavefrom
  Epaper_Write_Data(0x80);
  //
  Epaper_Write_Command(0x44);   // set RAM x address start/end, in page 35
  Epaper_Write_Data(x_start);   // RAM x address start at 00h;
  Epaper_Write_Data(x_end);     // RAM x address end at 0fh(15+1)*8->128
  Epaper_Write_Command(0x45);   // set RAM y address start/end, in page 35
  Epaper_Write_Data(y_start2);  // RAM y address start at 0127h;
  Epaper_Write_Data(y_start1);  // RAM y address start at 0127h;
  Epaper_Write_Data(y_end2);    // RAM y address end at 00h;
  Epaper_Write_Data(y_end1);    // ????=0

  Epaper_Write_Command(0x4E);  // set RAM x address count to 0;
  Epaper_Write_Data(x_start);
  Epaper_Write_Command(0x4F);  // set RAM y address count to 0X127;
  Epaper_Write_Data(y_start2);
  Epaper_Write_Data(y_start1);

  Epaper_Write_Command(0x24);  // Write Black and White image to RAM
  //Serial.println(PART_COLUMN * PART_LINE);
  for (i = 0; i < PART_COLUMN * PART_LINE / 8; i++) {
    Epaper_Write_Data(pgm_read_byte(&datas[i]));
  }
  //EPD_Part_Update();
}
/////////////////////////////////Single display////////////////////////////////////////////////

void EPD_WriteScreen_White(void) {
  int i;
  Epaper_Write_Command(0x24);  // write RAM for black(0)/white (1)
  for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
    Epaper_Write_Data(0xFF);
  }
  EPD_Update();
}

void EPD_WriteScreen_Black(void) {
  int i;
  Epaper_Write_Command(0x24);  // write RAM for black(0)/white (1)
  for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
    Epaper_Write_Data(0x00);
  }
  EPD_Update();
}
//////////////////////////////////END//////////////////////////////////////////////////


// 转换UTF-8字符串中的字符为点阵
unsigned char* extractUtf8Characters(const String& str, int* width) {
  const char* utf8_str = str.c_str();
  size_t len = str.length();
  unsigned char* dynamicArray;
  // 分配初始大小的内存
  int size = 1;
  *width = 0;
  dynamicArray = (unsigned char*)malloc(size * 16 * sizeof(unsigned char));

  for (size_t i = 0; i < len;) {
    char buffer[5] = { 0 };  // UTF-8字符最多占4个字节，多加1个字节用于存储字符串结束符
    unsigned char byte = utf8_str[i];

    if (byte < 0x80) {  // 1-byte character (ASCII)
      buffer[0] = utf8_str[i];
      i += 1;
      // 在每次循环中添加新元素到数组中
      if (findAscii(buffer[0]) != -1) {
        dynamicArray = (unsigned char*)realloc(dynamicArray, (size + 1) * 16 * sizeof(unsigned char));
        int ascii_index = findAscii(buffer[0]);  //索引的ascii字符行号
        // 循环遍历第n行的所有元素
        for (int j = 0; j < 16; j++) {
          dynamicArray[(size - 1) * 16 + j] = gImage_ascii_16[ascii_index][j];
        }
        size += 1;
        *width += 1;
      }
    } else if ((byte & 0xE0) == 0xC0) {  // 2-byte character
      buffer[0] = utf8_str[i];
      buffer[1] = utf8_str[i + 1];
      i += 2;

      // Serial.println("2-byte character");
      // Serial.println(buffer);
      // 在每次循环中添加新元素到数组中
      if (findHanzi(buffer) != -1) {
        dynamicArray = (unsigned char*)realloc(dynamicArray, (size + 2) * 16 * sizeof(unsigned char));
        int hanzi_index = findHanzi(buffer);  //索引的hanzi字符行号
        // 循环遍历第n行的所有元素
        for (int j = 0; i < 16; j++) {
          dynamicArray[(size - 1) * 16 + j] = gImage_chinese_16[hanzi_index * 2][j];
          dynamicArray[size * 16 + j] = gImage_chinese_16[hanzi_index * 2 + 1][j];
        }
        size += 2;
        *width += 2;
      }
      // 在每次循环中添加新元素到数组中
      if (findChineseSymbol(buffer) != -1) {
        dynamicArray = (unsigned char*)realloc(dynamicArray, (size + 2) * 16 * sizeof(unsigned char));
        int chinese_symbol_index = findChineseSymbol(buffer);  //索引的chinese_symbol字符行号
        // 循环遍历第n行的所有元素
        for (int j = 0; j < 16; j++) {
          dynamicArray[(size - 1) * 16 + j] = gImage_chinese_symbol_16[chinese_symbol_index * 2][j];
          dynamicArray[size * 16 + j] = gImage_chinese_symbol_16[chinese_symbol_index * 2 + 1][j];
        }
        size += 2;
        *width += 2;
      }
    } else if ((byte & 0xF0) == 0xE0) {  // 3-byte character (common for Chinese characters)
      buffer[0] = utf8_str[i];
      buffer[1] = utf8_str[i + 1];
      buffer[2] = utf8_str[i + 2];
      i += 3;
      // Serial.println("\n3-byte character:");
      // Serial.print(buffer);
      // Serial.print("\n findHanzi: ");
      // Serial.print(findHanzi(buffer));
      // Serial.print("\n findChineseSymbol: ");
      // Serial.println(findChineseSymbol(buffer));

      // 在每次循环中添加新元素到数组中
      if (findHanzi(buffer) != -1) {
        dynamicArray = (unsigned char*)realloc(dynamicArray, (size + 2) * 16 * sizeof(unsigned char));
        int hanzi_index = findHanzi(buffer);  //索引的hanzi字符行号
        // 循环遍历第n行的所有元素
        for (int j = 0; j < 16; j++) {
          dynamicArray[(size - 1) * 16 + j] = gImage_chinese_16[hanzi_index * 2][j];
          dynamicArray[size * 16 + j] = gImage_chinese_16[hanzi_index * 2 + 1][j];
        }
        size += 2;
        *width += 2;
      }
      // 在每次循环中添加新元素到数组中
      if (findChineseSymbol(buffer) != -1) {
        dynamicArray = (unsigned char*)realloc(dynamicArray, (size + 2) * 16 * sizeof(unsigned char));
        int chinese_symbol_index = findChineseSymbol(buffer);  //索引的chinese_symbol字符行号
        // 循环遍历第n行的所有元素
        for (int j = 0; j < 16; j++) {
          dynamicArray[(size - 1) * 16 + j] = gImage_chinese_symbol_16[chinese_symbol_index * 2][j];
          dynamicArray[size * 16 + j] = gImage_chinese_symbol_16[chinese_symbol_index * 2 + 1][j];
        }
        size += 2;
        *width += 2;
      }
    } else if ((byte & 0xF8) == 0xF0) {  // 4-byte character
      // Serial.println("4-byte character");
      // Serial.println(buffer);
      buffer[0] = utf8_str[i];
      buffer[1] = utf8_str[i + 1];
      buffer[2] = utf8_str[i + 2];
      buffer[3] = utf8_str[i + 3];
      i += 4;
    }
  }

  return dynamicArray;
}

// 获取天气预报
int getWeatherJson(JsonDocument* doc, String url, String district_id, String ak) {
  // 发起HTTPS GET请求
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient https;
    String re = url + "?district_id=" + district_id + "&data_type=all&ak=" + ak;
    https.begin(re);
    int httpCode = https.GET();
    if (Serial) Serial.printf("HTTP请求返回码: %d\n", httpCode);
    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
      String payload = https.getString();
      // 解析 JSON 字符串
      DeserializationError error = deserializeJson(*doc, payload);
    }
    https.end();

    return httpCode;
  }
  return 0;
}

// 索引汉字字模
int findHanzi(const char* targetHanzi) {
  int index = -1;  // 默认索引为-1，表示未找到
  // 查找汉字在map中的索引
  auto it = hanziMap.find(targetHanzi);
  if (it != hanziMap.end()) {
    index = it->second;
  }
  return index;
}

// 索引ASCII字模
int findAscii(const char targetAscii) {
  int index = -1;  // 默认索引为-1，表示未找到
  // 查找汉字在map中的索引
  auto it = ascii_Map.find(targetAscii);
  if (it != ascii_Map.end()) {
    index = it->second;
  }
  return index;
}
// 索引中文符号字模
int findChineseSymbol(const char* targetSymbolChinese) {
  int index = -1;  // 默认索引为-1，表示未找到
  // 查找汉字在map中的索引
  auto it = symbol_chinese_Map.find(targetSymbolChinese);
  if (it != symbol_chinese_Map.end()) {
    index = it->second;
  }
  return index;
}

// 天气图标转换
const unsigned char* findWeather(const char* weather) {
  if (strcmp(weather, "晴") == 0) {
    return gImage_sunny;
  }
  if (strcmp(weather, "多云") == 0) {
    return gImage_cloudy;
  }
  if (strcmp(weather, "暴雪") == 0) {
    return gImage_blizzard;
  }
  if (strcmp(weather, "暴雨") == 0) {
    return gImage_rainstorm;
  }
  if (strcmp(weather, "雷阵雨") == 0) {
    return gImage_thundershowers;
  }
  if (strcmp(weather, "沙尘暴") == 0) {
    return gImage_sandstorm;
  }
  if (strcmp(weather, "雾") == 0) {
    return gImage_fog;
  }
  if (strcmp(weather, "小雪") == 0) {
    return gImage_lightsnow;
  }
  if (strcmp(weather, "阴") == 0) {
    return gImage_overcast;
  }
  if (strcmp(weather, "雨夹雪") == 0) {
    return gImage_sleet;
  }
  if (strcmp(weather, "阵雪") == 0) {
    return gImage_snowshowers;
  }
  if (strcmp(weather, "阵雨") == 0) {
    return gImage_shower;
  }
  if (strcmp(weather, "大雪") == 0) {
    return gImage_heavysnow;
  }
  if (strcmp(weather, "大雨") == 0) {
    return gImage_heavyrain;
  }
  if (strcmp(weather, "小雨") == 0) {
    return gImage_lightrain;
  }
  if (strcmp(weather, "中雪") == 0) {
    return gImage_moderatesnow;
  }
  if (strcmp(weather, "中雨") == 0) {
    return gImage_moderaterain;
  }
  return gImage_unknow;
}

/***************缩放函数*******************/
void resizeImage(const unsigned char* src, int srcWidth, int srcHeight, unsigned char** dst, int dstWidth, int dstHeight) {
  if (dstHeight % 8 > 0)
    return;
  // 先进行行缩放
  double resizeHight = dstHeight * 1.0 / srcHeight;
  unsigned char* resizeH = (unsigned char*)malloc((srcWidth * dstHeight) / 8);
  if (resizeH == NULL) {
    if (Serial) Serial.printf("resizeH == NULL");
    return;
  }
  memset(resizeH, 0xFF, (srcWidth * dstHeight) / 8);
  if (Serial) Serial.printf("%d\n", (srcWidth * dstHeight) / 8);
  // 如果是等大或者缩小，直接按倍数移位
  if (resizeHight <= 1) {
    if (Serial) Serial.println("等大或者缩小");
    for (int x = 0; x < srcWidth; x++) {
      for (int y = 0; y < srcHeight; y++) {
        // 定位原bit位
        int srcByteIndex = (x * srcHeight + y) / 8;
        int srcBitIndex = 7 - y % 8;

        // 计算目标bit位
        int dstByteIndex = (int)((x * srcHeight + y * resizeHight) / 8);
        int dstBitIndex = 7 - (int)(y * resizeHight) % 8;

        // 取bit赋值
        if (!(src[srcByteIndex] & (1 << srcBitIndex))) {
          resizeH[dstByteIndex] &= ~(1 << dstBitIndex);
        }
      }
    }
  } else {
    // 如果是放大，涉及插值
    if (Serial) Serial.println("放大");
    for (int x = 0; x < srcWidth; x++) {
      for (int y = 0; y < srcHeight; y++) {
        // 锁定原bit位
        int srcByteIndex = (x * srcHeight + y) / 8;
        int srcBitIndex = 7 - y % 8;

        // 计算目标bit位
        int dstByteIndex = (int)((x * dstHeight + y * resizeHight) / 8);
        int dstBitIndex = 7 - (int)(y * resizeHight) % 8;

        // 取bit赋值
        if (!(src[srcByteIndex] & (1 << srcBitIndex))) {
          resizeH[dstByteIndex] &= ~(1 << dstBitIndex);
        }

        if (y < 1) continue;
        // 计算目标上一bit位，与本次bit位是否存在空位，及空位多少
        int preByteIndex = (int)((x * dstHeight + (y - 1) * resizeHight) / 8);
        int preBitIndex = 7 - (int)((y - 1) * resizeHight) % 8;
        int preBitOffset = (int)((y * resizeHight) - (int)((y - 1) * resizeHight));

        // 存在空位，补插值
        if (preBitOffset == 1) continue;

        byte dstBitValue = (resizeH[dstByteIndex] >> dstBitIndex) & B00000001;
        byte preBitValue = (resizeH[preByteIndex] >> preBitIndex) & B00000001;

        int offsetByteIndex, offsetBitIndex;
        if (dstBitValue ^= preBitValue) {
          int range = dstBitValue - preBitValue;
          int sum = dstBitValue + preBitValue;
          double step = range * 1.0 / preBitOffset;
          double avgValue = sum * 1.0 / 2;
          for (; preBitOffset > 1; preBitOffset--) {
            offsetByteIndex = (int)((x * dstHeight + y * resizeHight - preBitOffset + 1) / 8);
            offsetBitIndex = 7 - (int)(y * resizeHight - preBitOffset + 1) % 8;
            double value = (int)dstBitValue - step * (preBitOffset - 1);

            if (!value - avgValue > 1e-2) {
              resizeH[offsetByteIndex] &= ~(1 << offsetBitIndex);
            } else {
              resizeH[offsetByteIndex] |= (1 << offsetBitIndex);
            }
          }
        } else {
          for (; preBitOffset > 1; preBitOffset--) {
            offsetByteIndex = (int)((x * dstHeight + y * resizeHight - preBitOffset + 1) / 8);
            offsetBitIndex = 7 - (int)(y * resizeHight - preBitOffset + 1) % 8;

            if (!preBitValue) {
              resizeH[offsetByteIndex] &= ~(1 << offsetBitIndex);
            } else {
              resizeH[offsetByteIndex] |= (1 << offsetBitIndex);
            }
          }
        }
      }
    }
  }

  // 再进行列缩放
  double resizeWidth = dstWidth * 1.0 / srcWidth;
  *dst = (unsigned char*)malloc((dstWidth * dstHeight) / 8);
  if (*dst == NULL) {
    if (Serial) Serial.printf("*dst == NULL");
    return;
  }
  memset(*dst, 0xFF, (dstWidth * dstHeight) / 8);
  if (Serial) Serial.printf("%d\n", (dstWidth * dstHeight) / 8);
  // 如果是等大或者缩小，直接按倍数移位
  if (resizeWidth <= 1) {
    if (Serial) Serial.println("等大或者缩小");
    for (int y = 0; y < dstHeight / 8; y++) {
      for (int x = 0; x < srcWidth; x++) {
        // 计算源Byte位
        int srcByteIndex = x * dstHeight / 8 + y;
        // 计算目标Byte位
        int dstByteIndex = (int)(x * resizeWidth) * dstHeight / 8 + y;
        (*dst)[dstByteIndex] = resizeH[srcByteIndex];
      }
    }
  } else {
    // 如果是放大，涉及插值
    if (Serial) Serial.println("放大");
    for (int y = 0; y < dstHeight / 8; y++) {
      for (int x = 0; x < srcWidth; x++) {
        // 计算源Byte位
        int srcByteIndex = x * dstHeight / 8 + y;
        // 计算目标Byte位
        int dstByteIndex = (int)(x * resizeWidth) * dstHeight / 8 + y;

        (*dst)[dstByteIndex] = resizeH[srcByteIndex];

        if (x < 1) continue;

        // 计算目标上一Row，与本次Row是否存在空行，及空行多少
        int dstRow = (int)(x * resizeWidth);
        int preRow = (int)((x - 1) * resizeWidth);
        int preRowOffset = dstRow - preRow;

        // 存在空行，补插值
        if (preRowOffset == 1) continue;

        int overallStep = preRowOffset;

        for (; preRowOffset > 1; preRowOffset--) {
          for (int col = 0; col < dstHeight; col++) {
            int rowBitIndex = 7 - col % 8;
            int dstRowByteIndex = dstRow * dstHeight / 8 + col / 8;
            int preRowByteIndex = preRow * dstHeight / 8 + col / 8;
            int offsetByteIndex = (dstRow - preRowOffset + 1) * dstHeight / 8 + col / 8;

            byte dstRowBitValue = ((*dst)[dstRowByteIndex] >> rowBitIndex) & B00000001;
            byte preRowBitValue = ((*dst)[preRowByteIndex] >> rowBitIndex) & B00000001;

            if (dstRowBitValue ^= preRowBitValue) {
              int range = dstRowBitValue - preRowBitValue;
              int sum = dstRowBitValue + preRowBitValue;
              double step = range * 1.0 / overallStep;
              double avgValue = sum * 1.0 / 2;
              double value = (int)dstRowBitValue - step * (preRowOffset - 1);

              if (!value - avgValue > 1e-2) {
                (*dst)[offsetByteIndex] &= ~(1 << rowBitIndex);
              } else {
                (*dst)[offsetByteIndex] |= (1 << rowBitIndex);
              }
            } else {
              if (!preRowBitValue) {
                (*dst)[offsetByteIndex] &= ~(1 << rowBitIndex);
              } else {
                (*dst)[offsetByteIndex] |= (1 << rowBitIndex);
              }
            }
          }
        }
      }
    }
  }
}

/***************类实现*******************/
// 按YYYY/MM/DD HH:mm:ss格式返回当前时间字符串
int TimeClient_CTM::getYear() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  return timeinfo->tm_year + 1900;
}

int TimeClient_CTM::getMonth() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  return timeinfo->tm_mon + 1;
}

int TimeClient_CTM::getDay() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  return timeinfo->tm_mday;
}

int TimeClient_CTM::getHour() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  return timeinfo->tm_hour;
}

int TimeClient_CTM::getMinute() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  return timeinfo->tm_min;
}

int TimeClient_CTM::getSecond() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  return timeinfo->tm_sec;
}
String TimeClient_CTM::getCurrentDate() {

  // 计算当前本地日期
  char currentDate[11];
  sprintf(currentDate, "%04d/%02d/%02d", getYear(), getMonth(), getDay());
  return String(currentDate);
}

// 按YYYY/MM/DD HH:mm:ss格式返回当前时间字符串
String TimeClient_CTM::getCurrentTime() {
  // 计算当前本地时间
  char currentTime[9];
  sprintf(currentTime, "%02d:%02d:%02d", getHour(), getMinute(), getSecond());
  return String(currentTime);
}

// 获取星期几
String TimeClient_CTM::getDayOfWeek() {
  time_t t = now();
  tm* timeInfo = localtime(&t);
  const char* daysOfWeek[] = { "日", "一", "二", "三", "四", "五", "六" };
  return String(daysOfWeek[timeInfo->tm_wday]);
}

void TimeClient_CTM::fetchTime() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(apiUrl);
    int httpCode = http.GET();

    if (httpCode > 0) {
      String payload = http.getString();
      parseAndSetTime(payload);
    } else {
      if (Serial) Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  } else {
    if (Serial) Serial.println("WiFi not connected");
  }
}

void TimeClient_CTM::parseAndSetTime(const String& payload) {
  JsonDocument jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, payload);

  if (error) {
    if (Serial) Serial.print(F("deserializeJson() failed: "));
    if (Serial) Serial.println(error.c_str());
    return;
  }

  const char* datetime = jsonDoc["datetime"];
  struct tm tm;
  strptime(datetime, "%Y-%m-%dT%H:%M:%S%z", &tm);

  // 将时间调整为东八区（UTC+8）
  time_t t = mktime(&tm) + 8 * 3600;

  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);

  if (Serial) Serial.print("System time updated to: ");
  if (Serial) Serial.println(datetime);
}