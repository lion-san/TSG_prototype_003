//------------------------------------------------------------
//    姿勢制御フィルタリングプログラム
//                Arduino　IDE　1.6.11
//
//　　　Arduino　　　　　　　　LSM9DS1基板　
//　　　　3.3V　　　------　　　　3.3V
//　　　　GND       ------   　　 GND
//　　　　SCL       ------        SCL
//　　　　SDA       ------        SDA
//
//　センサーで取得した値をシリアルモニターに表示する
//
//　　　　
//----------------------------------------------------------//


#include <SPI.h>                                //SPIライブラリ
#include <Wire.h>                               //I2Cライブラリ
#include <SparkFunLSM9DS1.h>                  //LSM9DS1ライブラリ：https://github.com/sparkfun/LSM9DS1_Breakout
#include <SD.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SoftwareSerial.h>
#include <Kalman.h>               //KalmanFilter : https://github.com/TKJElectronics/KalmanFilter


//#define ADAddr 0x48//

#define LSM9DS1_M  0x1E                 // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B                // SPIアドレス設定 if SDO_AG is LOW

//#define PRINT_CALCULATED              //表示用の定義
//#define DEBUG_GYRO                    //ジャイロスコープの表示

#define DECLINATION -8.58               // Declination (degrees) in Boulder, CO.


#define RX 8                            //GPS用のソフトウェアシリアル
#define TX 9                            //GPS用のソフトウェアシリアル
#define SENTENCES_BUFLEN      82        // GPSのメッセージデータバッファの個数


#define MAX_FILES_DIGIT = 5             //5桁

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf 

//-------------------------------------------------------------------------
//[Global valiables]

LSM9DS1 imu;

//###############################################
//MicroSD 
//const int chipSelect = 4;//Arduino UNO
const int chipSelect = 10;//Arduino Micro
//###############################################

const int tact_switch = 7;//タクトスイッチ
boolean switchIs;
boolean switchOn;
boolean switchRelease;


String motionData;

///////////////////////カルマンフィルタ/////////////////////////////
Kalman kalmanX; // instances
Kalman kalmanY; // instances
unsigned long time;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double accX, accY, accZ; 
double gyroX, gyroY, gyroZ; 
float roll, pitch; 
////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------
//=== Global for GPS ===========================================
SoftwareSerial  g_gps( RX, TX );
char head[] = "$GPRMC";
char info[] = "$GPGGA";
char buf[10];
int SentencesNum = 0;                   // GPSのセンテンス文字列個数
byte SentencesData[SENTENCES_BUFLEN] ;  // GPSのセンテンスデータバッファ
boolean isReaded;                       //GPSの読み込みが完了したかどうか
String gpsData;                         //GPSの読み込みが完了データ
File dataFile;                          //SD CARD
boolean sdOpened = false;
//======================================================

void setup(void) {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  //=== SD Card Initialize ====================================
  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    return;
  }
  Serial.println(F("card initialized."));

  //=======================================================

  //タクトスイッチ
  pinMode(tact_switch, INPUT);
  switchIs = false;

  //=== LSM9DS1 Initialize =====================================
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress  = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())              //センサ接続エラー時の表示
  {
    Serial.println(F("Failed to communicate with LSM9DS1."));
    while (1)
      ;
  }
  //=======================================================


  //=== GPS用のソフトウェアシリアル有効化 =================
  setupSoftwareSerial();
  //=======================================================

  //=== カルマンフィルタの初期化処理 ================= 
  delay(100); // Wait for sensor to stabilize
  //初期値計算
  initCalmanFilter();
  //=======================================================
  
}

/**
 * 　==================================================================================================================================================
 * loop
 * ずっと繰り返される関数（何秒周期？）
 * 【概要】
 * 　10msでセンサーデータをサンプリング。
 * 　記録用に、100ms単位でデータ化。
 * 　蓄積したデータをまとめて、1000ms単位でSDカードにデータを出力する。
 * 　==================================================================================================================================================
 */
void loop(void) {

  switch(digitalRead(tact_switch)){

   case 0://ボタンを押した

          switchOn = true;

          break;

   case 1://ボタン押していない

           if(switchOn){

             //すでにOnなら、falseにする
             if(switchIs)
               switchIs = false;
             //すでにOffなら、trueにする
             else
               switchIs = true;


             switchOn = false;
            
           }

           break;

    default:
           break;
  }

  //スイッチの判定
  if(!switchIs){ //falseなら、ループする
    digitalWrite(13, 0);

    if (sdOpened) {
      sdcardClose();
    }
    else{
      ; 
    }
    return;
  }
  else{
    digitalWrite(13, 1);

    if (sdOpened) {
      ;
    }
    else{
      sdcardOpen();
    }

  }
  //END switch ============================================
  

  //GPS
  //▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
  char dt = 0 ;

  motionData = "";

    // センテンスデータが有るなら処理を行う
    if (g_gps.available()) {

        // 1バイト読み出す
        dt = g_gps.read() ;
        //Serial.write(dt);

        //Serial.write(dt);//Debug ALL
        // センテンスの開始
        if (dt == '$') SentencesNum = 0 ;
        
        if (SentencesNum >= 0) {
          
          // センテンスをバッファに溜める
          SentencesData[SentencesNum] = dt ;
          SentencesNum++ ;
             
          // センテンスの最後(LF=0x0Aで判断)
          if (dt == 0x0a || SentencesNum >= SENTENCES_BUFLEN) {
    
            SentencesData[SentencesNum] = '\0' ;
    
            //GPS情報の取得
            //getGpsInfo();

            //MotionSensorの値更新
            updateMotionSensors(false);
            
            // センテンスのステータスが"有効"になるまで待つ
            if ( gpsIsReady() )
            {
               // 有効になったら書込み開始
               //Serial.print("O:");
               //Serial.print( (char *)SentencesData );

               gpsData = String((char *)SentencesData );
               
               // read three sensors and append to the string:
               //記録用のセンサー値を取得
               motionData = updateMotionSensors(true);

               //SDカードへの出力
               writeDataToSdcard();


               return;
            }
          }
        }
      }

}


//==================================================================================================================================================


/**
 * sdcardOpen
 */
void sdcardOpen()
{


  // ファイル名決定
  String s;
  int fileNum = 0;
  char fileName[16];
  
  while(1){
    s = "LOG";
    if (fileNum < 10) {
      s += "00";
    } else if(fileNum < 100) {
      s += "0";
    }
    s += fileNum;
    s += ".TXT";
    s.toCharArray(fileName, 16);
    if(!SD.exists(fileName)) break;
    fileNum++;
  }


  dataFile = SD.open(fileName, FILE_WRITE);

  if(dataFile){
    Serial.println(fileName);
    sdOpened = true;
  }
  else
    Serial.println("fileError");

}

/**
 * sdcardClose
 */
void sdcardClose()
{

    dataFile.close();
    Serial.println(F("SD Closed."));
    sdOpened = false;

}




/**
 * writeDataToSdcard
 */
void writeDataToSdcard()
{

  //File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
    dataFile.print(gpsData);
    dataFile.print(motionData);

    //dataFile.close();
    
    // print to the serial port too:
    Serial.println(gpsData);
    Serial.println(motionData);
    Serial.println(F("================================"));
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(F("error opening datalog.txt"));
  }

}




/**
 * updateMotionSensors
 */
String updateMotionSensors(boolean print)
{
  
  //Read three sensors data on the memory
  readMotionSensors();
  
  //メモリ上の角度データの更新（前回値と今回値が考慮される）  
   return printAttitude(print) + "\n";
}




//--------------------　Motion DATA ------------------------------------
void readMotionSensors()
{
  imu.readGyro();
  imu.readAccel();
  imu.readMag();
}


//---------------------------------------------------------
/**
 * printAttitude
 * 取得したデータをシリアル出力する関数
 * gx : ジャイロスコープ X値
 * gy : ジャイロスコープ Y値
 * gz : ジャイロスコープ Z値
 * ax : 加速度センサー X値
 * ay : 加速度センサー Y値
 * az : 加速度センサー Z値
 * mx : 地磁気センサー X値
 * my : 地磁気センサー Y値
 * mz : 地磁気センサー Z値
 * print : 値を返すかどうか
 */

String printAttitude(boolean print)
{
  String output = "";

  accX = imu.ax; 
  accY = imu.ay; 
  accZ = imu.az; 

  
  gyroX = imu.gx; 
  gyroY = imu.gy; 
  gyroZ = imu.gz; 


  //時間の更新
  double dt = (double)(millis() - time) / 1000; // Calculate delta time  
  time = millis();
  Serial.print(dt);

#ifdef RESTRICT_PITCH // Eq. 25 and 26 
  roll  = atan2(accY, accZ) * RAD_TO_DEG;//+++++++++++++++++++++++ 
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
#else // Eq. 28 and 29 
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG; 
  pitch = atan2(-accX, accZ) * RAD_TO_DEG; 
#endif


  double gyroXrate = gyroX / 131.0; // Convert to deg/s 
  double gyroYrate = gyroY / 131.0; // Convert to deg/s 
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s 



#ifdef RESTRICT_PITCH 
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees 
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) { 
    kalmanX.setAngle(roll); 
    kalAngleX = roll; 
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter 
  
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); 
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees 
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) { 
    kalmanY.setAngle(pitch); 
    kalAngleY = pitch; 
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter 
  
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter 
#endif



float heading = 0;
/*
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
*/

    Serial.print(F("CalmanFilter: "));
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(kalAngleY);
    Serial.print(" ");
    Serial.println(kalAngleX);

    output += "CalmanFilter:"; 
    output += heading;
    output += " ";
    output += kalAngleY;
    output += " ";
    output += kalAngleX;
    
  return output;
  
}

/**
 *   initCalmanFilter
 */
void initCalmanFilter(){

  readMotionSensors();

  accX = imu.ax; 
  accY = imu.ay; 
  accZ = imu.az; 

#ifdef RESTRICT_PITCH // Eq. 25 and 26 
  roll  = atan2(accY, accZ) * RAD_TO_DEG; 
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
#else // Eq. 28 and 29 
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG; 
  pitch = atan2(-accX, accZ) * RAD_TO_DEG; 
#endif



  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch); // Set starting angle


  //時間の更新
  time = millis();
}


//===============================================
//
//      GPS系の処理
//
//===============================================

/**
 * setupSoftwareSerial
 * GPS用のソフトウェアシリアルの有効化
 */
void setupSoftwareSerial(){
  g_gps.begin(9600);
}


/**
 * getGpsInfo
 * $GPGGA　ヘッダから、衛星受信数や時刻情報を取得
 */
void getGpsInfo()
{
    int i, c;
    
    //$1ヘッダが一致
    if( strncmp((char *)SentencesData, info, 6) == 0 )
    {

      //コンマカウント初期化
      c = 1; 

      // センテンスの長さだけ繰り返す
      for (i=0 ; i<SentencesNum; i++) {
        if (SentencesData[i] == ','){
          
            c++ ; // 区切り文字を数える
    
            if ( c == 2 ) {
                 //Serial.println(F("----------------------------"));
                // Serial.println((char *)SentencesData);
                 Serial.print(F("Time:"));
                 Serial.println(readDataUntilComma(i+1));
                 continue;
            }
            else if ( c == 8 ) {
                // Serial.println((char *)SentencesData);
                 Serial.print(F("Number of Satelites:"));
                 Serial.println(readDataUntilComma(i+1));
                 continue;
            }
        }
      }
      
    }
}



/**
 * gpsIsReady
 * GPS情報が有効かどうかを判断
 * 項目3が"A"かどうかで判断
 */
boolean gpsIsReady()
{
    int i, c;
    
    //$1ヘッダが一致かつ,$3ステータスが有効＝A
    if( strncmp((char *)SentencesData, head, 6) == 0 )
    {

      //コンマカウント初期化
      c = 1; 

      // センテンスの長さだけ繰り返す
      for (i=0 ; i<SentencesNum; i++) {
        if (SentencesData[i] == ','){
              
              c++ ; // 区切り文字を数える
    
            if ( c == 3 ) {
                 //次のコンマまでのデータを呼び出し
                 if( strncmp("A", readDataUntilComma(i+1), 1) == 0 ){
                   Serial.print("O:");
                   return true;
                 }
                 else{
                   Serial.print("X:");
                   //Serial.print( (char *)SentencesData );
                   return false;
                 }
            }
        }
      }
      
    }

    return false;
}

/**
  * readDataUntilComma
  */
char* readDataUntilComma(int s)
{
  int i, j;

  j = 0;
  //初期化
  memset(buf,0x00,sizeof(buf)) ;

  //終了条件
  //次のコンマが出現or特定文字*（チェックサム)が出現
  for (i = s; i < SentencesNum; i++)
  {
    if(( SentencesData[i] == ',') || (SentencesData[i] == '*')){
      buf[j] = '\0';
      return buf;
    }
    else{
      //バッファーのオーバフローをチェック
      if( j < sizeof(buf) ) {
        buf[j] = SentencesData[i];
        j++;
      }
      else{//エラー処理
        int x;
        for(x = 0; x < sizeof(buf); x++)
          buf[x] = 'X';
          return buf;
      }
      
    }
  }
  
}

