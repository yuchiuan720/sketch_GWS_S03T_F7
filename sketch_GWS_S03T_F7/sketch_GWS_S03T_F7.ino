#include <Servo.h>
#include <EEPROM.h>

Servo myServo;
const int servoPin = 3;  // SERVO 輸出引腳
const int voltPin = A0;  // VOLT 模式偵測引腳
const int openPin1 = A1; // OPEN 模式偵測引腳 1（接待測物至 GND）
const int openPin2 = A2; // OPEN 模式偵測引腳 2（接待測物至 GND）

// EEPROM 記憶體地址分配
const int EEPROM_VALID_ADDR = 0;   
const int EEPROM_A1_ADDR    = 1;
const int EEPROM_A2_ADDR    = 3;  
const int EEPROM_SPEED_ADDR = 5;   
const int EEPROM_WAIT2_ADDR = 7;  
const int EEPROM_WAIT1_ADDR = 11; 
const int EEPROM_COUNT_ADDR = 15;  
const int EEPROM_MODE_ADDR  = 20;  // 儲存檢測模式的地址
const byte EEPROM_MAGIC_NUM = 43;  // 變更 Magic Number 以更新結構

// 檢測模式枚舉
enum DetectMode {
  MODE_VOLT = 0,
  MODE_OPEN = 1
};
DetectMode currentMode = MODE_VOLT; // 預設為電壓偵測模式

// 控制與狀態變數
bool isRunning = false;
unsigned long runCount = 0;    
bool isDetectEnabled = true; // 💡【補回】失效監測總開關（true=啟動保護, false=忽略保護）

// 雙端點角度變數 (支援 0 ~ 200 度)
int angle1 = 0;                 
int angle2 = 180;              
unsigned long targetRunLimit = 0; 
bool isLimitMode = false;        

// 速度與保護參數
int stepDelay = 6;              
int currentAngle = 0;          
int moveDirection = 1;          // 1 代表角度遞增，-1 代表角度遞減
bool isAttached = false;        

// 停留時間參數
unsigned long delayAtTarget = 1000;
unsigned long delayAtZero   = 500;  

// 起中終點歷史狀態暫存變數
int savedA2_Pin1 = 0;
int savedA2_Pin2 = 0;
float savedA2_Volt = 0.0;

int savedMid_Angle = 90;
int savedMid_Pin1 = 0;
int savedMid_Pin2 = 0;
float savedMid_Volt = 0.0;

// =================【極簡完美硬體校正公式】=================
void writeRealAngle(int realAngle) {
  if (isAttached) {
    if (realAngle < 0) realAngle = 0;
    if (realAngle > 200) realAngle = 200;

    int usTarget = 500 + (realAngle * 9);
    myServo.writeMicroseconds(usTarget);
  }
}
// ==========================================================

// =================【通用型類比讀取換算 + 4次採樣抗噪濾波】=================
float getPinVoltage(int pin) {
  long sum = 0;
  for(int i = 0; i < 4; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50); // 確保 ADC 電容充電穩定
  }
  float raw = sum / 4.0;
  return (raw * 5.0) / 1023.0;
}

float getVoltage() {
  return getPinVoltage(voltPin);
}
// ==========================================================

// 更新引腳模式（依據當前偵測模式切換）
void updatePinModes() {
  if (currentMode == MODE_OPEN) {
    pinMode(openPin1, INPUT_PULLUP);
    pinMode(openPin2, INPUT_PULLUP);
    Serial.println(F("SYSTEM: 已啟動 A1, A2 內部上拉，切換至開路偵測 (OPEN MODE)"));
  } else {
    pinMode(openPin1, INPUT);
    pinMode(openPin2, INPUT);
    Serial.println(F("SYSTEM: 切換至電壓偵測 (VOLT MODE)"));
  }
}

void ensureAttached() {
  if (!isAttached) {
    myServo.attach(servoPin, 400, 2500);
    isAttached = true;
  }
}

void resetSystem() {
  isRunning = false;
  isLimitMode = false;
  targetRunLimit = 0;
  runCount = 0;
  if (isAttached) {
    myServo.detach();
    isAttached = false;
  }
}

void initializeDirection() {
  if (angle1 < angle2) moveDirection = 1;
  else moveDirection = -1;
}

void loadSettings() {
  byte magic = EEPROM.read(EEPROM_VALID_ADDR);
  if (magic == EEPROM_MAGIC_NUM) {
    EEPROM.get(EEPROM_A1_ADDR, angle1);
    EEPROM.get(EEPROM_A2_ADDR, angle2);
    EEPROM.get(EEPROM_SPEED_ADDR, stepDelay);
    EEPROM.get(EEPROM_WAIT2_ADDR, delayAtTarget);
    EEPROM.get(EEPROM_WAIT1_ADDR, delayAtZero);
    EEPROM.get(EEPROM_COUNT_ADDR, runCount);
    
    byte modeByte;
    EEPROM.get(EEPROM_MODE_ADDR, modeByte);
    currentMode = (DetectMode)modeByte;
    
    initializeDirection();
    Serial.println(F("EEPROM: 成功偵測到歷史紀錄，已自動加載參數！"));
  } else {
    Serial.println(F("EEPROM: 未偵測到歷史紀錄，使用系統預設參數。"));
  }
  updatePinModes(); 
}

void saveSettings() {
  EEPROM.write(EEPROM_VALID_ADDR, EEPROM_MAGIC_NUM);
  EEPROM.put(EEPROM_A1_ADDR, angle1);
  EEPROM.put(EEPROM_A2_ADDR, angle2);
  EEPROM.put(EEPROM_SPEED_ADDR, stepDelay);
  EEPROM.put(EEPROM_WAIT2_ADDR, delayAtTarget);
  EEPROM.put(EEPROM_WAIT1_ADDR, delayAtZero);
  EEPROM.put(EEPROM_COUNT_ADDR, runCount);
  EEPROM.put(EEPROM_MODE_ADDR, (byte)currentMode); 
  Serial.println(F("EEPROM: 所有參數與當前計數已成功保存！(SAVE SUCCESS)"));
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("=== GWS S03T 串口控制系統（ATE 自動失效監測版） ==="));
  
  loadSettings();
  ensureAttached();
  
  // 【修正】初始物理角度應與軟體變數 currentAngle 一致，避免開機角度暴衝
  currentAngle = angle1; 
  // writeRealAngle(currentAngle);
  writeRealAngle(90);
  
  Serial.println(F("指令說明:"));
  Serial.println(F("  GO                  - 啟動/繼續 自動來回測試"));
  Serial.println(F("  NOGO                - 平滑復歸90度並切斷訊號"));
  Serial.println(F("  SAVE                - 手動將當前參數與進度存入晶片"));
  Serial.println(F("  RUN [次數]          - 設定指定測試次數並立刻開始"));
  Serial.println(F("  CLR                 - 暫停測試、清除限制並歸零"));
  Serial.println(F("  TO [角度]           - 平滑移動到指定手動角度 (支援 0~200)"));
  Serial.println(F("  SET [角度1] [角度2] - 設定擺動雙端點 (支援 0~200，可顛倒大小)"));
  Serial.println(F("  SPEED [ms]          - 設定每度延遲時間 (ms)"));
  Serial.println(F("  WAIT [A2_ms] [A1_ms] - 設定兩端停留時間"));
  Serial.println(F("  MODE [VOLT/OPEN]    - 切換失效監測模式 (電壓/開路斷開)")); 
  Serial.println(F("  VOLT / OPEN         - 快速切換監測模式")); 
  Serial.println(F("  DETECT ON / OFF     - 啟動或關閉失效中斷保護")); 
  Serial.println(F("  R                   - 類比讀取 A0, A1, A2 即時電壓")); 
  Serial.println(F("  GET                 - 查詢目前所有參數與測試進度"));
  Serial.println(F("========================================="));
}

void loop() {
  // 1. 核心串口防錯讀取
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();
    if (inputStr.length() == 0) return;
    
    if (inputStr.equals("GO")) {
      ensureAttached(); isRunning = true;
      if (isLimitMode) { Serial.print(F("SYSTEM: RESUME RUN MODE. Target: ")); Serial.println(targetRunLimit); } 
      else { Serial.println(F("SYSTEM: START AUTO-MODE")); }
    } 
    else if (inputStr.equals("NOGO")) {
      isRunning = false; // 停止自動運行
      ensureAttached();  // 確保馬達訊號線還接著
      
      // 1. 讓馬達平滑滑行到「接近」90度的變數位置
      while (currentAngle != 90) {
        if (currentAngle < 90) currentAngle++;
        else currentAngle--;
        
        writeRealAngle(currentAngle); // 依然跟隨變數平滑滑行，防止爆衝
        delay(stepDelay); 
      }
      
      // 2. 滑行結束後，強制補刀！對馬達下達絕對精準的 90 度指令
      writeRealAngle(90); 
      delay(150); // 給馬達 150 毫秒的物理時間，微調修正最後那 1~2 度的物理誤差
      
      // 3. 確定完全到位後，再安全切斷控制訊號
      if (isAttached) {  
        myServo.detach(); 
        isAttached = false; 
      }
      Serial.println(F("SYSTEM: SMOOTH TO 90 & PAUSED (SIGNAL DETACHED)"));
    }
    else if (inputStr.equals("SAVE")) {
      saveSettings();
    }
    else if (inputStr.startsWith("RUN ")) {
      long runInput = inputStr.substring(4).toInt();
      if (runInput > 0) {
        targetRunLimit = runInput; isLimitMode = true;
        ensureAttached(); isRunning = true;
        Serial.print(F("SYSTEM: START RUN MODE. Target loops: ")); Serial.println(targetRunLimit);
      } else { Serial.println(F("ERROR: RUN count must be greater than 0")); }
    }
    else if (inputStr.equals("CLR")) {
      resetSystem();
      Serial.println(F("SYSTEM: RESET. Run limit cleared and count reset to 0."));
    }
    else if (inputStr.startsWith("MODE ")) {
      String modeParam = inputStr.substring(5); modeParam.toUpperCase();
      if (modeParam.equals("VOLT")) { currentMode = MODE_VOLT; updatePinModes(); Serial.println(F("SUCCESS: Mode changed to VOLT")); } 
      else if (modeParam.equals("OPEN")) { currentMode = MODE_OPEN; updatePinModes(); Serial.println(F("SUCCESS: Mode changed to OPEN")); } 
      else { Serial.println(F("ERROR: Invalid mode. Use 'MODE VOLT' or 'MODE OPEN'")); }
    }
    else if (inputStr.equals("VOLT")) { currentMode = MODE_VOLT; updatePinModes(); Serial.println(F("SUCCESS: Mode changed to VOLT")); }
    else if (inputStr.equals("OPEN")) { currentMode = MODE_OPEN; updatePinModes(); Serial.println(F("SUCCESS: Mode changed to OPEN")); }
    // 💡【補回】監測總開關串口指令處理
    else if (inputStr.equals("DETECT ON")) {
      isDetectEnabled = true;
      Serial.println(F("SYSTEM: 失效監測已【啟動】(PROTECTION ENABLED)"));
    }
    else if (inputStr.equals("DETECT OFF")) {
      isDetectEnabled = false;
      Serial.println(F("⚠️ SYSTEM WARNING: 失效監測已【關閉】(PROTECTION DISABLED)"));
    }
    else if (inputStr.equals("R")) {
      Serial.println(F("--- REAL-TIME VOLTAGE READ ---"));
      Serial.print(F("  A0 (VOLT Pin) : ")); Serial.print(getPinVoltage(voltPin)); Serial.println(F(" V"));
      Serial.print(F("  A1 (OPEN Pin1): ")); Serial.print(getPinVoltage(openPin1)); Serial.println(F(" V"));
      Serial.print(F("  A2 (OPEN Pin2): ")); Serial.print(getPinVoltage(openPin2)); Serial.println(F(" V"));
      if (currentMode == MODE_OPEN) { Serial.println(F("  [NOTE] 系統當前為 OPEN 模式，A1/A2 電壓包含內部上拉電阻電位")); }
      Serial.println(F("------------------------------"));
    }
    else if (inputStr.startsWith("TO ")) {
      int toAngle = inputStr.substring(3).toInt();
      if (toAngle >= 0 && toAngle <= 200) { 
        isRunning = false; ensureAttached();
        Serial.print(F("SYSTEM: Moving to ")); Serial.print(toAngle); Serial.println(F(" deg..."));
        while (currentAngle != toAngle) {
          if (Serial.available() > 0) {
            String checkStr = Serial.readStringUntil('\n'); checkStr.trim();
            if (checkStr.equals("NOGO") || checkStr.equals("CLR")) {
              myServo.detach(); isAttached = false;
              Serial.println(F("SYSTEM: EMERGENCY STOP DURING 'TO'")); return;
            }
          }
          if (currentAngle < toAngle) currentAngle++; else currentAngle--;
          writeRealAngle(currentAngle); delay(stepDelay);
        }
        Serial.println(F("SUCCESS: Arrived."));
      } else { Serial.println(F("ERROR: Angle must be between 0 and 200")); }
    }
    else if (inputStr.startsWith("SET ")) {
      String params = inputStr.substring(4); int spaceIndex = params.indexOf(' ');
      if (spaceIndex != -1) {
        int a1_input = params.substring(0, spaceIndex).toInt(); int a2_input = params.substring(spaceIndex + 1).toInt();
        if (a1_input >= 0 && a1_input <= 200 && a2_input >= 0 && a2_input <= 200 && a1_input != a2_input) {
          angle1 = a1_input; angle2 = a2_input; initializeDirection();
          Serial.print(F("SUCCESS: Range set from ")); Serial.print(angle1); Serial.print(F(" to ")); Serial.println(angle2);
        } else { Serial.println(F("ERROR: Angles must be 0~200 and Angle1 != Angle2")); }
      } else { Serial.println(F("ERROR: Use format 'SET [Angle1] [Angle2]'")); }
    } 
    else if (inputStr.startsWith("SPEED ")) {
      int speedInput = inputStr.substring(6).toInt();
      if (speedInput > 0) { stepDelay = speedInput; Serial.print(F("SUCCESS: stepDelay = ")); Serial.print(stepDelay); Serial.println(F(" ms")); } 
      else { Serial.println(F("ERROR: Speed > 0")); }
    }
    else if (inputStr.startsWith("WAIT ")) {
      String params = inputStr.substring(5); int spaceIndex = params.indexOf(' ');
      if (spaceIndex != -1) {
        long tWait = params.substring(0, spaceIndex).toInt(); long zWait = params.substring(spaceIndex + 1).toInt();
        if (tWait >= 0 && zWait >= 0) {
          delayAtTarget = tWait; delayAtZero = zWait;
          Serial.print(F("SUCCESS: WAIT Angle2: ")); Serial.print(delayAtTarget); Serial.print(F("ms, Angle1: ")); Serial.print(delayAtZero); Serial.println(F("ms"));
        }
      }
    }
    else if (inputStr.equals("GET")) {
      Serial.println(F("--- CURRENT SETTINGS & PROGRESS ---"));
      Serial.print(F("  Status: ")); if (isRunning) Serial.println(isLimitMode ? F("RUNNING (LIMIT MODE)") : F("RUNNING (INFINITE)")); else Serial.println(F("PAUSED / STOPPED"));
      Serial.print(F("  Detection Mode: ")); Serial.println(currentMode == MODE_VOLT ? F("VOLT (電壓監測)") : F("OPEN (開路監測)"));
      // 💡【優化】在狀態面板中同步加入開關狀況顯示
      Serial.print(F("  Detection Switch: ")); Serial.println(isDetectEnabled ? F("ON (保護中)") : F("OFF (已忽略保護)"));
      Serial.print(F("  Current Progress (COUNT): ")); Serial.print(runCount); if (isLimitMode) { Serial.print(F(" / ")); Serial.println(targetRunLimit); } else { Serial.println(F(" / 無限")); }
      
      if (currentMode == MODE_VOLT) { Serial.print(F("  DUT Battery Voltage: ")); Serial.print(getVoltage()); Serial.println(F(" V")); } 
      else {
        Serial.print(F("  Pin Status (A1 / A2): ")); 
        Serial.print(digitalRead(openPin1) == HIGH ? F("OPEN(斷開)") : F("GND(正常)")); Serial.print(F(" / ")); Serial.println(digitalRead(openPin2) == HIGH ? F("OPEN(斷開)") : F("GND(正常)"));
      }
      Serial.print(F("  Current Position: ")); Serial.print(currentAngle); Serial.println(F(" deg"));
      Serial.print(F("  Auto Test Route (SET): ")); Serial.print(angle1); Serial.print(F(" -> ")); Serial.print(angle2); Serial.println(F(" deg"));
      Serial.print(F("  Step Delay (SPEED): ")); Serial.print(stepDelay); Serial.println(F(" ms"));
      Serial.print(F("  Wait (Angle2/Angle1): ")); Serial.print(delayAtTarget); Serial.print(F("ms / ")); Serial.print(delayAtZero); Serial.println(F("ms"));
      Serial.println(F("-----------------------------------"));
    }
  }

  // 2. 自動平滑馬達運轉邏輯
  if (isRunning) {
    bool isFailed = false;
    float currentVolt = 0.0;
    int tmp1 = LOW, tmp2 = LOW;
    
    // 💡【核心修改】只有在監測功能啟動（isDetectEnabled == true）時才進行健康度抽查
    if (isDetectEnabled) {
      if (currentMode == MODE_VOLT) {
        currentVolt = getVoltage();
        if (currentVolt < 0.1) { isFailed = true; }
      } else {
        tmp1 = digitalRead(openPin1);
        tmp2 = digitalRead(openPin2);
        if (tmp1 == HIGH || tmp2 == HIGH) { isFailed = true; }
      }
    } else {
      // 即使關閉保護，背景依然刷新 currentVolt，確保後續中點與端點 LOG 的資料新鮮度
      if (currentMode == MODE_VOLT) currentVolt = getVoltage();
    }

    // 如果未跨出步伐前就已經掛了，且監測是開啟的，立刻觸發終止保護
    if (isDetectEnabled && isFailed) {
      Serial.println(F("\n========================================="));
      if (currentMode == MODE_VOLT) {
        Serial.print(F("⚠️⚠️⚠️ 警告：偵測到待測物電壓歸零 (")); Serial.print(currentVolt); Serial.println(F("V) ⚠️⚠️⚠️"));
      } else {
        Serial.print(F("⚠️⚠️⚠️ 警告：偵測到待測物迴路斷開！狀態 -> A1: "));
        Serial.print(tmp1 == HIGH ? F("斷開 ") : F("正常 ")); Serial.print(F(", A2: ")); Serial.println(tmp2 == HIGH ? F("斷開") : F("正常"));
      }
      Serial.print(F("FATAL: 待測物已失效！終止於第 ")); Serial.print(runCount); Serial.println(F(" 次往返計數。"));
      resetSystem();
      Serial.println(F("SYSTEM: ATE 已緊急安全停機。"));
      Serial.println(F("========================================="));
      return;
    }

    // 確定安全，計算步進角度並移動 1 度
    currentAngle += moveDirection;
    int maxAngle = max(angle1, angle2);
    int minAngle = min(angle1, angle2);
    if (currentAngle > maxAngle) currentAngle = maxAngle;
    if (currentAngle < minAngle) currentAngle = minAngle;
    
    writeRealAngle(currentAngle);
    delay(stepDelay); 

    // 【動態捕捉：正中央點的角度與狀態】
    int midPoint = (angle1 + angle2) / 2;
    if (currentAngle == midPoint) {
      savedMid_Angle = midPoint;
      if (currentMode == MODE_VOLT) {
        savedMid_Volt = getVoltage();
      } else {
        savedMid_Pin1 = digitalRead(openPin1);
        savedMid_Pin2 = digitalRead(openPin2);
      }
    }

    // 【到達 Angle2 端點】（記錄狀態，暫不輸出）
    if (currentAngle == angle2) {
      if ((angle1 < angle2 && moveDirection == 1) || (angle1 > angle2 && moveDirection == -1)) {
        if (currentMode == MODE_VOLT) {
          savedA2_Volt = getVoltage();
        } else {
          savedA2_Pin1 = digitalRead(openPin1);
          savedA2_Pin2 = digitalRead(openPin2);
        }

        // 不卡死的端點 2 停留時間
        unsigned long startTime = millis();
        while (millis() - startTime < delayAtTarget) { 
          if (Serial.available() > 0) return;
        }
        moveDirection = -moveDirection; // 調轉方向
      }
    }
    // 【回到 Angle1 端點】（完成一次完整往返，進行單行濃縮 LOG 輸出）
    else if (currentAngle == angle1) {
      if ((angle1 < angle2 && moveDirection == -1) || (angle1 > angle2 && moveDirection == 1)) {
        
        // 不卡死的端點 1 停留時間
        unsigned long startTime = millis();
        while (millis() - startTime < delayAtZero) { 
          if (Serial.available() > 0) return;
        }
        moveDirection = -moveDirection; // 調轉方向
        
        runCount++; // 只有完美回到終點，計數器才可靠的 +1
        
        // 輸出當趟完整的歷史捕捉單行 LOG
        Serial.print(F("COUNT: ")); Serial.print(runCount);
        if (currentMode == MODE_VOLT) {
          float finalVolt = getVoltage();
          Serial.print(F(", ")); Serial.print(angle1); Serial.print(F("度[")); Serial.print(finalVolt); Serial.print(F("V]"));
          Serial.print(F(", ")); Serial.print(savedMid_Angle); Serial.print(F("度[")); Serial.print(savedMid_Volt); Serial.print(F("V]"));
          Serial.print(F(", ")); Serial.print(angle2); Serial.print(F("度[")); Serial.print(savedA2_Volt); Serial.println(F("V]"));
        } else {
          int currentA1_Pin1 = digitalRead(openPin1);
          int currentA1_Pin2 = digitalRead(openPin2);
          Serial.print(F(", ")); Serial.print(angle1); Serial.print(F("度[")); Serial.print(currentA1_Pin1); Serial.print(F(",")); Serial.print(currentA1_Pin2); Serial.print(F("]"));
          Serial.print(F(", ")); Serial.print(savedMid_Angle); Serial.print(F("度[")); Serial.print(savedMid_Pin1); Serial.print(F(",")); Serial.print(savedMid_Pin2); Serial.print(F("]"));
          Serial.print(F(", ")); Serial.print(angle2); Serial.print(F("度[")); Serial.print(savedA2_Pin1); Serial.print(F(",")); Serial.print(savedA2_Pin2); Serial.println(F("]"));
        }

        // 每 10 次自動寫入 EEPROM 保護進度
        if (runCount % 10 == 0) { 
          EEPROM.put(EEPROM_COUNT_ADDR, runCount);
        }

        // 計次模式的到達判定
        if (isLimitMode && (runCount >= targetRunLimit)) {
          Serial.println(F("========================================="));
          Serial.print(F("SYSTEM: TARGET REACHED! Completed ")); Serial.print(runCount); Serial.println(F(" runs."));
          resetSystem();
          Serial.println(F("SYSTEM: Auto-executed CLR. Ready for next command."));
          Serial.println(F("========================================="));
        }
      }
    }
  } // End of if(isRunning)
}