#include <MAVLink.h>
#include <Arduino.h>
#include "myUARTSensor.h"
#include "myUART.h"
#include "Orientation.h"

#define PIXHAWK_SERIAL Serial1
#define Serial_xsens Serial2
#define NMEA_OUT_Serial Serial3
#define NMEA_IN_Serial Serial4

XsensUart xsens(Serial_xsens);
int64_t time_offset = 0;
uint64_t xsens_pre_time = 0;
uint8_t current_Xsens_mode = MODE_AHRS;
uint8_t current_output_mode = OUT_MODE_BIN;
bool is_run = false;  
bool is_debug = false;
bool ISR_flag_xsens = false, ISR_flag_NMEA = false;
bool USB_Setting_mode = false;
bool enable_input = true;
bool time_sync_initialized = false;  // 新增：時間同步初始化標記
bool gps_connected = false;          // GPS 連接狀態
unsigned long last_gps_data_time = 0; // 最後收到 GPS 數據的時間
my_data_u4 latestXsensTime;
unsigned long last_sync_millis = 0;         // 上一次時間同步的時間點（ms）
const unsigned long SYNC_INTERVAL = 5000;   // 每 5 秒重新同步一次，減少干擾（單位：ms）

// 平滑 time_offset：使用滑動平均法
const int OFFSET_BUF_SIZE = 5;
int64_t time_offset_buffer[OFFSET_BUF_SIZE] = {0};
int offset_index = 0;
bool offset_buffer_full = false;

// 底下加入自動同步新增進入 loop()
void syncPX4Time() {
  if (millis() - last_sync_millis >= SYNC_INTERVAL) {
    uint64_t px4_time = getPX4Time(false);
    if (px4_time > 0) {
      int64_t new_offset = px4_time - uint64_t(latestXsensTime.ulong_val) * 1e2;

      // 將新 offset 放入緩衝區
      time_offset_buffer[offset_index] = new_offset;
      offset_index++;

      if (offset_index >= OFFSET_BUF_SIZE) {
        offset_index = 0;
        offset_buffer_full = true;
      }

      int count = offset_buffer_full ? OFFSET_BUF_SIZE : offset_index;
      int64_t sum = 0;
      for (int i = 0; i < count; i++) {
        sum += time_offset_buffer[i];
      }
      time_offset = sum / count;

      last_sync_millis = millis();
      Serial.print("[SYNC] Updated smoothed time_offset = ");
      Serial.println(time_offset);
    }
  }
}
void SERCOM2_Handler() {
  Serial_xsens.IrqHandler();
  ISR_flag_xsens = true;
}

void SERCOM3_Handler() {
  NMEA_IN_Serial.IrqHandler();
  ISR_flag_NMEA = true;
}

void setup() {
  myUART_init();
  // while(!Serial);
  is_debug = checkUSBSerial();
  checkPX4CON();
  
  // initialize xsens mit-680
  xsens.setDataRate(30);
  setXsensPackage();
  xsens.ToMeasurementMode();

  delay(100);
  is_run = true;
}

void loop() { 
  // Check GPS connection status (every 10 seconds)
  static unsigned long last_gps_check = 0;
  if (millis() - last_gps_check > 10000) {
    if (gps_connected && (millis() - last_gps_data_time > 5000)) {
      gps_connected = false;
      Serial.println("[GPS] Connection lost - no data for 5 seconds");
    }
    last_gps_check = millis();
  }

  if (ISR_flag_xsens && is_run){
    readXsens();
    ISR_flag_xsens = false;
  }

  if (ISR_flag_NMEA){
    if (current_output_mode == OUT_MODE_NMEA){
      printNMEAWithModifiedTimestamp(NMEA_IN_Serial, PIXHAWK_SERIAL, false);  
    }
    else{
      printNMEAWithModifiedTimestamp(NMEA_IN_Serial, NMEA_OUT_Serial, current_output_mode == OUT_MODE_GNSS);
    }
    
    ISR_flag_NMEA = false;
    
  }

  if (Serial.available()){ 
    if (USB_Setting_mode && (current_output_mode == OUT_MODE_XBUS)){
      checkXBUS_CMD(Serial);
    }
    
    else{
      // checkSTR_CMD(Serial.readString());
      checkSTR_CMD(readCurrentBytes(Serial));
    }
  }

  if (PIXHAWK_SERIAL.available() && enable_input){
    if (current_output_mode == OUT_MODE_BIN && PIXHAWK_SERIAL.available() >= 0x20) {
      mavlink_message_t msg;
      mavlink_status_t status;
      while (PIXHAWK_SERIAL.available()) {
        uint8_t c = PIXHAWK_SERIAL.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
          enable_input = false; 

          current_output_mode = OUT_MODE_ML_ODOM;
          current_Xsens_mode = MODE_INS_PAV_QUT;
          send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_PAV_QUT...");
          send2Serial(PIXHAWK_SERIAL, "Set MAVLINK ODOMETRY(331) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_GPS_RAW;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_RAW_INT(24) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_GPS_IN;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_INPUT(232) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_VISO;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION POSITION ESTIMATOR(102) Output...");

          // current_Xsens_mode = MODE_INS_UTC;
          // current_output_mode = OUT_MODE_NMEA;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_UTC...");
          // send2Serial(PIXHAWK_SERIAL, "Set NMEA Output...");

          // current_Xsens_mode = MODE_INS_PAV_QUT;
          // current_output_mode = OUT_MODE_VEC;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to MODE_INS_PAV_QUT...");
          // send2Serial(PIXHAWK_SERIAL, "Set Vector BIN Output...");

          setXsensPackage();
          sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
          break;
        }
      }
    }
    
    else if (current_output_mode == OUT_MODE_XBUS){
      uint8_t buffer[LEN_XBUS];
      uint16_t idx = 0;
      while (PIXHAWK_SERIAL.available() && idx < LEN_XBUS) {
        buffer[idx++] = PIXHAWK_SERIAL.read();
      }

      if (idx > 0) { 
        if (idx >= 6){
          if (buffer[0] == 'C' && buffer[1] == 'O' && buffer[2] == 'N' && buffer[3] == 'F' && buffer[4] == 'I' && buffer[5] == 'G'){
            current_output_mode = OUT_MODE_CONFIG;
            send2Serial(PIXHAWK_SERIAL, "Enter CONFIG mode");
          }
        }
        if (current_output_mode == OUT_MODE_XBUS) { Serial_xsens.write(buffer, idx); }  
      }
    }

    else{ 
      // checkSTR_CMD(PIXHAWK_SERIAL.readString()); 
      checkSTR_CMD(readCurrentBytes(PIXHAWK_SERIAL));
    }
  }
}

void readXsens(){
  if (Serial_xsens.available()){
    if (current_output_mode == OUT_MODE_XBUS){
      uint8_t buffer[LEN_XBUS];
      uint16_t idx = 0;
      while (Serial_xsens.available() && idx < LEN_XBUS) {
        buffer[idx++] = Serial_xsens.read();
      }
      if (idx > 0) { 
        if (USB_Setting_mode) { Serial.write(buffer, idx); }
        else { PIXHAWK_SERIAL.write(buffer, idx); }
      }
    }

    else{
      my_data_3f omg, acc, ori, mag, vel;
      my_data_4f qut;
      my_data_u4 XsensTime, temp, pressure, hei, status;
      my_data_2d latlon;
      my_data_u2 xsens_counter;
      UTC_TIME utc_time;

      latlon.float_val[0] = 0;
      latlon.float_val[1] = 0;
      hei.float_val = 0;
      vel.float_val[0] = 0;
      vel.float_val[1] = 0;
      vel.float_val[2] = 0;
        
      xsens.getMeasures(MTDATA2);
      xsens.parseData(&xsens_counter, &XsensTime, &omg, &acc, &mag, &pressure, &vel, &latlon, &hei, &ori, &qut, &status, &temp);
      
      if (xsens.getDataStatus() == DATA_OK) {
        // Validate sensor data before processing
        bool data_valid = true;
        
        // Check timestamp monotonicity
        if (XsensTime.ulong_val <= xsens_pre_time && xsens_pre_time != 0) {
          Serial.println("Warning: Non-monotonic timestamp detected");
          data_valid = false;
        }
        
        // Validate angular velocity (reasonable range: -10 to +10 rad/s)
        for (int i = 0; i < 3; i++) {
          if (abs(omg.float_val[i]) > 10.0) {
            Serial.print("Warning: Excessive angular velocity detected: ");
            Serial.println(omg.float_val[i]);
            data_valid = false;
          }
        }
        
        // Validate quaternion norm (should be close to 1.0)
        float quat_norm = sqrt(qut.float_val[0]*qut.float_val[0] + 
                              qut.float_val[1]*qut.float_val[1] + 
                              qut.float_val[2]*qut.float_val[2] + 
                              qut.float_val[3]*qut.float_val[3]);
        if (abs(quat_norm - 1.0) > 0.1) {
          Serial.print("Warning: Invalid quaternion norm: ");
          Serial.println(quat_norm);
          data_valid = false;
        }
        
        if (!data_valid) {
          Serial.println("Skipping invalid sensor data");
          return;
        }
        
        // 更新最新的 Xsens 時間用於同步
        latestXsensTime = XsensTime;
        
        // 時間同步初始化 - 加入更完整的檢查
        if (!time_sync_initialized) {
          uint64_t px4_time = getPX4Time(false);
          if (px4_time > 0 && px4_time > XsensTime.ulong_val * 1e2) {
            time_offset = px4_time - XsensTime.ulong_val * 1e2;
            time_sync_initialized = true;
            Serial.print("[INIT] Time sync initialized with offset: ");
            Serial.println((long long)time_offset);
            
            // Initialize the offset buffer
            for (int i = 0; i < OFFSET_BUF_SIZE; i++) {
              time_offset_buffer[i] = time_offset;
            }
            offset_buffer_full = true;
          }
        }
        
        char buffer[128];
        int index = 0;
        
        // MAVLink output - 每次都發送，確保 30Hz
        if (current_output_mode == OUT_MODE_ML_ODOM && current_Xsens_mode == MODE_INS_PAV_QUT){
          sendMAVLink_Odometry(XsensTime, latlon, hei, vel, qut, omg, status);
        } 
        else if (current_output_mode == OUT_MODE_ML_GPS_RAW && current_Xsens_mode == MODE_INS){
          sendMAVLink_GPS_RAW_INT(XsensTime, latlon, hei, vel, ori, omg, status);
        }
        else if (current_output_mode == OUT_MODE_ML_GPS_IN && current_Xsens_mode == MODE_INS){
          sendMAVLink_GPS_INPUT(XsensTime, latlon, hei, vel, ori, omg, status);
        }
        else if (current_output_mode == OUT_MODE_ML_VISO && current_Xsens_mode == MODE_INS){
          sendMAVLink_VISION_POSITION(XsensTime, latlon, hei, vel, ori, omg, status);
        }
 
        else if (current_output_mode == OUT_MODE_STR) {
          if (current_Xsens_mode == MODE_INS_PAV_QUT){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, latlon.float_val, 2, 7);
            index = appendValue2Str(buffer, 128, index, hei.float_val, 3);
            index = appendValues2Str(buffer, 128, index, vel.float_val, 3, 2);
            index = appendValues2Str(buffer, 128, index, qut.float_val, 4, 4);
            xsens.getFilterStatus(status, true, PIXHAWK_SERIAL);
            PIXHAWK_SERIAL.println(buffer);
          }
          else if (current_Xsens_mode == MODE_INS){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, latlon.float_val, 2, 7);
            index = appendValue2Str(buffer, 128, index, hei.float_val, 3);
            index = appendValues2Str(buffer, 128, index, vel.float_val, 3, 2);
            index = appendValues2Str(buffer, 128, index, ori.float_val, 3, 2);
            xsens.getFilterStatus(status, true, PIXHAWK_SERIAL);
            PIXHAWK_SERIAL.println(buffer);
          } 
          else if (current_Xsens_mode == MODE_AHRS){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, omg.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, acc.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, mag.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, ori.float_val, 3, 2);
            PIXHAWK_SERIAL.println(buffer);
          } 
          else if (current_Xsens_mode == MODE_IMU){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, omg.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, acc.float_val, 3, 4);
            index = appendValue2Str(buffer, 128, index, temp.float_val, 1);
            PIXHAWK_SERIAL.println(buffer);
          }
          else { Serial.println("String output doesn't supportan this mode yet!"); }
        }
        
        else if (current_output_mode == OUT_MODE_BIN) {
          my_data_u4 output_time;
          uint8_t new_temp_bin[4];
          output_time.ulong_val = XsensTime.ulong_val * 0.1;
          for (int i=0;i<4;i++){ new_temp_bin[i] = temp.bin_val[3-i]; }  // inverse the order of temperature bytes

          if (current_Xsens_mode == MODE_AHRS){
            uint8_t buffer[52];
            memcpy(buffer, output_header, 4);
            memcpy(buffer + 4, omg.bin_val, 12);
            memcpy(buffer + 16, acc.bin_val, 12);
            memcpy(buffer + 28, new_temp_bin, 4);  // MSB
            memcpy(buffer + 32, output_time.bin_val, 4);
            memcpy(buffer + 36, ori.bin_val, 12);
            MyCRC().calCRC(buffer, 52);
            PIXHAWK_SERIAL.write(buffer, 52);
          }
          else if (current_Xsens_mode == MODE_INS){
            uint8_t buffer[76];
            memcpy(buffer, output_header, 4);
            memcpy(buffer + 4, omg.bin_val, 12);
            memcpy(buffer + 16, acc.bin_val, 12);
            memcpy(buffer + 28, new_temp_bin, 4);  // MSB
            memcpy(buffer + 32, output_time.bin_val, 4);
            memcpy(buffer + 36, ori.bin_val, 12);
            memcpy(buffer + 48, latlon.bin_val, 8);
            memcpy(buffer + 56, hei.bin_val, 4);
            memcpy(buffer + 60, vel.bin_val, 12);
            MyCRC().calCRC(buffer, 76);
            PIXHAWK_SERIAL.write(buffer, 76);
          }
          else {
            PIXHAWK_SERIAL.println("Binary output doesn't supported this mode yet!");
          }
        }

        else if (current_output_mode == OUT_MODE_VEC && current_Xsens_mode == MODE_INS_PAV_QUT){
          // length of header = 10 bytes
          // length of payload = 102 bytes
          // length of CRC = 2 bytes
          uint8_t message[10+102+2] = {
            0xFA,         // Sync header
            0x36,         // TimeGroup (1), ImuGroup (2), AttitudeGroup (4), InsGroup (5) -> 0b00110110
            0x00, 0x01,   // TimeGroup:     TimeStartup (0) -> 0x00, 0b00000001
            0x01, 0x30,   // ImuGroup:      Temp (4), Pres (5), MAG (8) -> 0b00000001, 0b00110000
            0x00, 0x84,   // AttitudeGroup: Quaternion (2), LinearAccelNed (7) -> 0x00, 0b10000100
            0x06, 0x13,   // InsGroup:      InsStatus (0), PosLla (1), VelNed (4), PosU (9), VelU (10) -> 0b00000110, 0b00010011
          };
          my_data_u8 time_nano;
          time_nano.ulong_val = XsensTime.ulong_val * 1e2;
          
          uint8_t ins_state[2];
          xsens.getINSStatus(status, ins_state);

          my_data_u8 lat, lon, alt;
          lat.ulong_val = latlon.float_val[0];
          lon.ulong_val = latlon.float_val[1];
          alt.ulong_val = hei.float_val;

          my_data_3f pos_u;
          pos_u.float_val[0] = 1.0f;
          pos_u.float_val[1] = 1.0f;
          pos_u.float_val[2] = 1.0f;

          my_data_u4 vel_u;
          vel_u.float_val = 0.05;

          uint8_t *ptr = message + 10;
          write_big_endian(ptr, time_nano.bin_val, 8);  ptr += 8;
          write_big_endian(ptr, temp.bin_val, 4);       ptr += 4;
          write_big_endian(ptr, pressure.bin_val, 4);   ptr += 4;
          write_big_endian(ptr, mag.bin_val, 12);       ptr += 12;
          write_big_endian(ptr, qut.bin_val, 16);       ptr += 16;
          memset(ptr, 0, 12);                           ptr += 12;  // LinearAccelNed
          write_big_endian(ptr, temp.bin_val, 4);       ptr += 4;
          memcpy(ptr, ins_state, 2);                    ptr += 2;
          write_big_endian(ptr, lat.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, lon.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, alt.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, vel.bin_val, 12);       ptr += 12;
          write_big_endian(ptr, pos_u.bin_val, 12);     ptr += 12;
          write_big_endian(ptr, vel_u.bin_val, 4);      ptr += 4;
          uint16_t crc = calculateCRC(message, 10 + 102);
          *(ptr++) = (crc >> 8) & 0xFF;
          *(ptr++) = crc & 0xFF;
          PIXHAWK_SERIAL.write(message, 10+102+2);
        }

        if (XsensTime.ulong_val - xsens_pre_time >= 33333 || xsens_pre_time == 0) {  // ✅ 約 30Hz
          xsens_pre_time = XsensTime.ulong_val;
          if (current_output_mode != OUT_MODE_CONFIG){
            Serial.println(XsensTime.ulong_val * 1e-4, 3);
            // xsens.getFilterStatus(status, true, Serial);
          }
          else{
            send2Serial(PIXHAWK_SERIAL, "CONFIG");
          }
        }
      }
      else if(xsens.getDataStatus() == DATA_WARNING){
        Serial.print("Warning: ");
        xsens.PrintMessage();
        PIXHAWK_SERIAL.print("Warning: ");
        xsens.PrintMessage(PIXHAWK_SERIAL);
      }
    }
  }
}

void sendMAVLink_Odometry(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_4f &qut, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_odometry_t odom = {};
    odom.time_usec = uint64_t(XsensTime.ulong_val) * 1e2 + time_offset;
    
    odom.frame_id = MAV_FRAME_GLOBAL;
    odom.child_frame_id = MAV_FRAME_LOCAL_NED;
    // Transform latlon to ENU (removed hardcoded test coordinates)
    
    float COV_MEAS;
    if (xsens.getFilterStatus(status) > 0){
      COV_MEAS = 1e-3;

      // Correct coordinate mapping: x=latitude, y=longitude, z=height
      odom.x = latlon.float_val[0];  // latitude
      odom.y = latlon.float_val[1];  // longitude
      odom.z = hei.float_val;
      odom.vx = vel.float_val[0];
      odom.vy = vel.float_val[1];
      odom.vz = vel.float_val[2];
    } else{
      // When filter status is poor, use zero values instead of hardcoded test data
      COV_MEAS = 1e-1;  // Higher uncertainty when filter is not working
      odom.x = 0;
      odom.y = 0;
      odom.z = 0;
      odom.vx = 0;
      odom.vy = 0;
      odom.vz = 0;
    } 

    odom.q[0] = qut.float_val[0];
    odom.q[1] = qut.float_val[1];
    odom.q[2] = qut.float_val[2];
    odom.q[3] = qut.float_val[3];
    odom.rollspeed = omg.float_val[0];
    odom.pitchspeed = omg.float_val[1];
    odom.yawspeed = omg.float_val[2];

    // Improved covariance settings - more realistic values for EKF
    odom.pose_covariance[0]  = COV_MEAS;     // x position variance
    odom.pose_covariance[7]  = COV_MEAS;     // y position variance  
    odom.pose_covariance[14] = COV_MEAS;     // z position variance
    odom.pose_covariance[21] = 1e-4;         // roll variance (more realistic)
    odom.pose_covariance[28] = 1e-4;         // pitch variance (more realistic)
    odom.pose_covariance[35] = 1e-3;         // yaw variance (higher for better EKF performance)
    
    odom.velocity_covariance[0]  = COV_MEAS * 2;  // vx variance
    odom.velocity_covariance[7]  = COV_MEAS * 2;  // vy variance
    odom.velocity_covariance[14] = COV_MEAS * 2;  // vz variance
    odom.velocity_covariance[21] = 1e-4;          // roll rate variance (more realistic)
    odom.velocity_covariance[28] = 1e-4;          // pitch rate variance (more realistic)
    odom.velocity_covariance[35] = 1e-3;          // yaw rate variance (higher for stability)
    
    odom.reset_counter = 0;
    odom.quality = 100;
    odom.estimator_type = MAV_ESTIMATOR_TYPE_NAIVE;

    mavlink_message_t msg;
    mavlink_msg_odometry_encode(1, 200, &msg, &odom);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    PIXHAWK_SERIAL.write(buffer, len);
}

void sendMAVLink_GPS_RAW_INT(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_gps_raw_int_t gps_input;
    gps_input.time_usec = uint64_t(XsensTime.ulong_val) * 1e2 + time_offset;
    gps_input.fix_type = min(3, xsens.getFilterStatus(status));
    gps_input.lat = int32_t(latlon.float_val[0] * 1e7);
    gps_input.lon = int32_t(latlon.float_val[1] * 1e7);
    gps_input.alt = hei.float_val;
    gps_input.vel = sqrt(vel.float_val[0] * vel.float_val[0] + vel.float_val[1] * vel.float_val[1]);
    // Convert yaw from radians to centidegrees, ensure positive angle (0-36000)
    float yaw_deg = ori.float_val[2] * 180.0 / M_PI;
    if (yaw_deg < 0) yaw_deg += 360.0;
    gps_input.yaw = (uint16_t)(yaw_deg * 100);

    // gps_input.satellites_visible = 7;
    // gps_input.fix_type = 3;
    // gps_input.lat = int32_t(23.5 * 1e7);
    // gps_input.lon = int32_t(121.0 * 1e7);
    // gps_input.alt = 1230;
    // gps_input.vel = 5;

    gps_input.vel_acc = 1;
    gps_input.h_acc = 1;
    gps_input.v_acc = 1;
    gps_input.eph = 1; // 水平精度 (米)
    gps_input.epv = 1; // 垂直精度 (米)gps_input.cog = cog / 100.0f;  // 地面航向 (度)

    
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_encode(1, 200, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    PIXHAWK_SERIAL.write(buffer, len);
}


void sendMAVLink_GPS_INPUT(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_gps_input_t gps_input;
    gps_input.time_usec = uint64_t(XsensTime.ulong_val) * 1e2 + time_offset;
    gps_input.fix_type = min(3, xsens.getFilterStatus(status));
    gps_input.lat = int32_t(latlon.float_val[0] * 1e7);
    gps_input.lon = int32_t(latlon.float_val[1] * 1e7);
    gps_input.alt = hei.float_val;
    gps_input.vn = vel.float_val[0];
    gps_input.ve = vel.float_val[1];
    gps_input.vd = vel.float_val[2];
    
    // gps_input.fix_type = 3;
    // gps_input.lat = int32_t(23.5 * 1e7);
    // gps_input.lon = int32_t(121.0 * 1e7);
    // gps_input.alt = 1.23;
    // gps_input.vn = 0.1;
    // gps_input.ve = -0.1;
    // gps_input.vd = 0.05;
    // Convert yaw from radians to centidegrees, ensure positive angle (0-36000)
    float yaw_deg = ori.float_val[2] * 180.0 / M_PI;
    if (yaw_deg < 0) yaw_deg += 360.0;
    gps_input.yaw = (uint16_t)(yaw_deg * 100);
    gps_input.satellites_visible = 7;

    gps_input.speed_accuracy = 1e-6;
    gps_input.horiz_accuracy = 1e-6;
    gps_input.vert_accuracy = 1e-6;
    gps_input.hdop = 1e-2;; // 水平精度 (米)
    gps_input.vdop = 1e-2;; // 垂直精度 (米)gps_input.cog = cog / 100.0f;  // 地面航向 (度)

    
    mavlink_message_t msg;
    mavlink_msg_gps_input_encode(1, 200, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    PIXHAWK_SERIAL.write(buffer, len);
}

void sendMAVLink_VISION_POSITION(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_vision_position_estimate_t viso = {};
    viso.usec = uint64_t(XsensTime.ulong_val) * 1e2 + time_offset;

    // Transform latlon to ENU (removed hardcoded test coordinates)
    
    float COV_MEAS;
    if (xsens.getFilterStatus(status) > 0){
      COV_MEAS = 1e-3;

      // Correct coordinate mapping: x=latitude, y=longitude, z=height
      viso.x = latlon.float_val[0];  // latitude
      viso.y = latlon.float_val[1];  // longitude
      viso.z = hei.float_val;
    } else{
      // When filter status is poor, use zero values instead of hardcoded test data
      COV_MEAS = 1e-1;  // Higher uncertainty when filter is not working
      viso.x = 0;
      viso.y = 0;
      viso.z = 0;
    } 

    viso.pitch = ori.float_val[0];
    viso.roll = ori.float_val[1];
    viso.yaw = ori.float_val[2];
    // More realistic covariance values for vision position estimate
    viso.covariance[0]  = COV_MEAS;     // x position variance
    viso.covariance[6]  = COV_MEAS;     // y position variance
    viso.covariance[11] = COV_MEAS;     // z position variance
    viso.covariance[15] = 1e-4;         // roll variance
    viso.covariance[18] = 1e-4;         // pitch variance
    viso.covariance[20] = 1e-3;         // yaw variance (higher for EKF stability)
    viso.reset_counter = 0;

    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_encode(1, 200, &msg, &viso);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    PIXHAWK_SERIAL.write(buffer, len);
}


uint64_t getPX4Time(bool is_print) {
  mavlink_message_t msg_send, msg_recv;
  mavlink_timesync_t ts_send, ts_recv;

  // 初始化時間戳
  ts_send.tc1 = 0;
  ts_send.ts1 = micros();

  // 編碼並發送 TIMESYNC 消息
  mavlink_msg_timesync_encode(1, 200, &msg_send, &ts_send);
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg_send);
  PIXHAWK_SERIAL.write(buffer, len);

  unsigned long start_time = millis();
  bool received = false;

  // 等待回應，最多等待 100 毫秒
  while (millis() - start_time < 100) {
    if (PIXHAWK_SERIAL.available() > 0) {
      uint8_t c = PIXHAWK_SERIAL.read();
      mavlink_status_t status;
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg_recv, &status)) {
        if (msg_recv.msgid == MAVLINK_MSG_ID_TIMESYNC) {
          mavlink_msg_timesync_decode(&msg_recv, &ts_recv);
          received = true;
          if (is_print) {
            Serial.print("ts1: ");
            Serial.println(ts_recv.ts1 * 0.001, 3); // 微秒轉毫秒
            Serial.print("tc1: ");
            Serial.println(ts_recv.tc1 * 1e-9, 3); // 微秒轉毫秒
          }
          return ts_recv.tc1;
        }
      }
    }
  }

  if (!received && is_print) {
    Serial.println("No TIMESYNC response received.");
  }
  return 0;
}

void setXsensPackage(){
  send2Serial(PIXHAWK_SERIAL, "MODE: " + String(current_Xsens_mode));
  xsens.ToConfigMode();
  // xsens.getFW();
  xsens.reqPortConfig();

  if (current_output_mode == OUT_MODE_ML_ODOM) {
    xsens.setAngleUnitDeg(false);
    xsens.setFrameENU(false);
    xsens.INS_PAV_QUT();
    current_Xsens_mode = MODE_INS_PAV_QUT;
  } 
  else if (current_output_mode == OUT_MODE_ML_GPS_RAW) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_ML_GPS_IN) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_ML_VISO) {
    xsens.setAngleUnitDeg(false);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_NMEA){
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INS_UTC();
    current_Xsens_mode = MODE_INS_UTC;
  }
  else if (current_output_mode == OUT_MODE_VEC) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INS_PAV_QUT();
    current_Xsens_mode = MODE_INS_PAV_QUT;
  } 
  else {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(true);
    if (current_Xsens_mode == MODE_INS) { xsens.INSData(); } 
    else if (current_Xsens_mode == MODE_AHRS) { xsens.AHRSData(); }
    else if (current_Xsens_mode == MODE_IMU) { xsens.IMURawMeas(); }
    else if (current_Xsens_mode == MODE_GNSS_RAW) { xsens.GNSSRawMeas(); } 
    else { send2Serial(PIXHAWK_SERIAL, "This mode is not supported yet!"); }
  }

  xsens.InitMT();
}

void checkPX4CON(){
  int counter = 0;
  for (int i=0;i<5;i++){
    if (getPX4Time(false)) { break; }
    counter++;
    delay(100);
  }
  if (counter == 5){ Serial.println("PX4CON not connected!"); } 
  else { Serial.println("PX4CON connected!"); }
}

bool checkUSBSerial(){
  for (int i=0;i<5;i++){
    if (!Serial) { delay(10); }
    else { 
      delay(3000);
      Serial.println("Connect to PC");
      return true; 
    }
  }
  return false;
}

void checkXBUS_CMD(Stream &port){
  uint8_t buffer[LEN_XBUS];
  uint16_t idx = 0;
  while (port.available() && idx < LEN_XBUS) {
    buffer[idx++] = port.read();
  }
  if (idx > 0) { 
    if (idx >= 6){
      if (buffer[0] == 'C' && buffer[1] == 'O' && buffer[2] == 'N' && buffer[3] == 'F' && buffer[4] == 'I' && buffer[5] == 'G'){
        current_output_mode = OUT_MODE_CONFIG;
        send2Serial(PIXHAWK_SERIAL, "Enter CONFIG mode");
      }
    }
    if (current_output_mode == OUT_MODE_XBUS) { Serial_xsens.write(buffer, idx); }  
  }
}

void checkSTR_CMD(String command){
  command.trim();
  send2Serial(PIXHAWK_SERIAL, "Command: " + command);
  if (command == "AHRS"){
    current_Xsens_mode = MODE_AHRS;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to AHRS...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);  
  }
  else if (command == "AHRS_QUT"){
    current_Xsens_mode = MODE_AHRS_QUT;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to AHRS_QUT...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "INS"){
    current_Xsens_mode = MODE_INS;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "INS_PAV_QUT"){
    current_Xsens_mode = MODE_INS_PAV_QUT;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_PAV_QUT...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "IMU"){
    current_Xsens_mode = MODE_IMU;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to IMU...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "GNSS_RAW"){
    current_output_mode = MODE_GNSS_RAW;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to GNSS_RAW...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "ML_ODOM"){
    current_output_mode = OUT_MODE_ML_ODOM;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK ODOMETRY(331) Output...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "ML_GNSS"){
    current_output_mode = OUT_MODE_ML_GPS_RAW;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_INPUT(232) Output...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "ML_VISO"){
    current_output_mode = OUT_MODE_ML_VISO;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION_POSITION_ESTIMATE(102) Output...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "VEC"){
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-300*58");
    Serial.println("Send: $VNRRG,01,VN-300*58");
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-310*58");
    Serial.println("Send: $VNRRG,01,VN-310*58");

    current_output_mode = OUT_MODE_VEC;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION_POSITION_ESTIMATE(102) Output...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "$VNRRG,01*72"){
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-300*58");
    Serial.println("Send: $VNRRG,01,VN-300*58");

    PIXHAWK_SERIAL.println("$VNRRG,01,VN-310*58");
    Serial.println("Send: $VNRRG,01,VN-310*58");
  }

  else if (command == "BIN"){
    current_output_mode = OUT_MODE_BIN;
    send2Serial(PIXHAWK_SERIAL, "Set Binary Output...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "STR"){
    current_output_mode = OUT_MODE_STR;
    send2Serial(PIXHAWK_SERIAL, "Set String Output...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "XBUS"){
    current_output_mode = OUT_MODE_XBUS;
    send2Serial(PIXHAWK_SERIAL, "Set XBUS protocal...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "NMEA" || command.startsWith("GPGGA")){
  // else if (command == "NMEA"){
    current_output_mode = OUT_MODE_NMEA;
    send2Serial(PIXHAWK_SERIAL, "Set NMEA Mode ...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }

  else if (command == "GNSS_TEST"){
    current_output_mode = OUT_MODE_GNSS;
    send2Serial(PIXHAWK_SERIAL, "Set GNSS Test Mode ...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "USB"){
    USB_Setting_mode = true;
    current_output_mode = OUT_MODE_CONFIG;
    send2Serial(PIXHAWK_SERIAL, "Set USB Configuration Mode ...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "USB_OUT"){
    send2Serial(PIXHAWK_SERIAL, "Leave USB Configuration Mode ...");
    USB_Setting_mode = false;
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "CONFIG") {
    current_output_mode = OUT_MODE_CONFIG;
    send2Serial(PIXHAWK_SERIAL, "Set CONFIG Mode...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "RESET") {
    send2Serial(PIXHAWK_SERIAL, "Reseting...");
    xsens.ToConfigMode();
    xsens.reset();
    xsens.InitMT();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }

  else if (command == "CALI_GYRO") {
    xsens.caliGyro(10, &PIXHAWK_SERIAL);
  }

  else if (command == "INIT_XSENS" && USB_Setting_mode){
    Serial.println("Start to config Xsens MTI-680");
    MyQuaternion::Quaternion qut(0, radians(180), radians(-90));
    xsens.ToConfigMode();
    xsens.setAlignmentRotation(qut.getQ()[3], qut.getQ()[0], qut.getQ()[1], qut.getQ()[2]);
    xsens.setGnssReceiverSettings(115200, 2, 1);
    xsens.InitMT();
    xsens.ToMeasurementMode();
    delay(500);
  }

  else if (command == "INIT_LOCOSYS" && USB_Setting_mode){
    Serial.println("Start to config LOCOSYS");
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,2,1*3D");   // enable GSA
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,4,1*3B");   // enable RMC
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,0,1*3F");   // enable GGA
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,8,1*37");   // enable GST
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,3,0*3D");   // disable GSV
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,5,0*3B");   // disable VTG
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
  }
}

void send2Serial(HardwareSerial &port, const char* str){
  Serial.println(str);
  port.println(str);
}

void send2Serial(HardwareSerial &port, const String str){
  Serial.println(str);
  port.println(str);
}

void printNMEAWithModifiedTimestamp(HardwareSerial &input_port, HardwareSerial &output_port, bool is_gnss_test) {
  String nmea_line = "";
  unsigned long start_time = millis();
  
  // Read NMEA sentence with timeout
  while (input_port.available() && (millis() - start_time < 100)) {
    char c = input_port.read();
    if (c == '\n' || c == '\r') {
      if (nmea_line.length() > 6) {  // Valid NMEA minimum length
        processNMEASentence(nmea_line, output_port, is_gnss_test);
      }
      nmea_line = "";
    } else {
      nmea_line += c;
    }
  }
  
  // Handle any remaining data
  if (nmea_line.length() > 6) {
    processNMEASentence(nmea_line, output_port, is_gnss_test);
  }
}

void processNMEASentence(String nmea_sentence, HardwareSerial &output_port, bool is_gnss_test) {
  // Validate NMEA checksum
  if (!validateNMEAChecksum(nmea_sentence)) {
    if (is_debug) {
      Serial.println("Invalid NMEA checksum: " + nmea_sentence);
    }
    return;
  }
  
  // Update GPS connection status
  last_gps_data_time = millis();
  if (!gps_connected) {
    gps_connected = true;
    Serial.println("[GPS] Connected - receiving NMEA data");
  }
  
  // For GNSS test mode, just forward the data
  if (is_gnss_test) {
    output_port.println(nmea_sentence);
    return;
  }
  
  // Parse and modify timestamp if needed
  String modified_sentence = modifyNMEATimestamp(nmea_sentence);
  output_port.println(modified_sentence);
  
  if (is_debug) {
    Serial.println("NMEA: " + modified_sentence);
  }
}

bool validateNMEAChecksum(String nmea_sentence) {
  int asterisk_pos = nmea_sentence.indexOf('*');
  if (asterisk_pos == -1 || asterisk_pos >= nmea_sentence.length() - 2) {
    return false;  // No checksum found
  }
  
  // Calculate checksum
  uint8_t calculated_checksum = 0;
  for (int i = 1; i < asterisk_pos; i++) {  // Start after '$'
    calculated_checksum ^= nmea_sentence.charAt(i);
  }
  
  // Get provided checksum
  String checksum_str = nmea_sentence.substring(asterisk_pos + 1);
  uint8_t provided_checksum = strtol(checksum_str.c_str(), NULL, 16);
  
  return calculated_checksum == provided_checksum;
}

String modifyNMEATimestamp(String nmea_sentence) {
  // For now, just return the original sentence
  // TODO: Implement timestamp modification based on time_offset
  return nmea_sentence;
}

bool checkLOCOSYS_ACK(HardwareSerial &port) {
  unsigned long start_time = millis();
  String response = "";
  
  // Wait for acknowledgment with timeout (3 seconds)
  while (millis() - start_time < 3000) {
    if (port.available()) {
      char c = port.read();
      if (c == '\n' || c == '\r') {
        if (response.length() > 0) {
          // Check for LOCOSYS acknowledgment patterns
          if (response.indexOf("$PACK") >= 0 || 
              response.indexOf("ACK") >= 0 || 
              response.indexOf("OK") >= 0) {
            if (is_debug) {
              Serial.println("LOCOSYS ACK: " + response);
            }
            return true;
          }
          response = "";
        }
      } else {
        response += c;
      }
    }
    delay(10);
  }
  
  if (is_debug) {
    Serial.println("LOCOSYS ACK timeout");
  }
  return false;
}

String readCurrentBytes(HardwareSerial &port) {
  String result = "";
  unsigned long start_time = millis();
  
  // Read available bytes with timeout
  while (port.available() && (millis() - start_time < 1000)) {
    char c = port.read();
    if (c == '\n' || c == '\r') {
      if (result.length() > 0) {
        break;  // Complete line received
      }
    } else {
      result += c;
    }
  }
  
  return result;
}

void sendProcessingDataAndStartMeas(HardwareSerial &port){
  xsens.ToConfigMode();
  xsens.reset();
  xsens.InitMT();

  delay(100);
  Serial.println();
  port.println();
  xsens.ToMeasurementMode();
}

// Calculates the 16-bit CRC for the given ASCII or binary message.
unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc = (unsigned char)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (unsigned char)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}

