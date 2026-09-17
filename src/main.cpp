#include <Arduino.h>
#include <MAVLink.h>

// Wiring (3.3 V UART, shared GND):
//   ESP32-S3 GPIO17 (TX) -> flight controller TELEM RX
//   ESP32-S3 GPIO18 (RX) <- flight controller TELEM TX
//   GND                  -> GND
// Typical FC telem baud is 57600 (ArduPilot SERIAL1 / PX4 TELEM).

static constexpr int MAV_RX_PIN = 18;
static constexpr int MAV_TX_PIN = 17;
static constexpr uint32_t MAV_BAUD = 57600;
static constexpr uint32_t DEBUG_BAUD = 115200;

static constexpr uint8_t GCS_SYS_ID = 255;
static constexpr uint8_t GCS_COMP_ID = MAV_COMP_ID_MISSIONPLANNER;

static constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
static constexpr uint32_t TELEMETRY_PRINT_PERIOD_MS = 1000;
static constexpr uint32_t STREAM_REFRESH_PERIOD_MS = 10000;
static constexpr int32_t MESSAGE_INTERVAL_US = 1000000;  // 1 Hz
static constexpr uint32_t FC_TIMEOUT_MS = 3000;

struct Telemetry {
  bool has_heartbeat = false;
  bool has_sys_status = false;
  bool has_attitude = false;
  bool has_position = false;
  bool has_gps = false;
  bool has_vfr = false;
  bool has_battery = false;

  uint8_t sysid = 0;
  uint8_t compid = 0;
  uint8_t type = 0;
  uint8_t autopilot = 0;
  uint8_t base_mode = 0;
  uint8_t system_status = 0;
  uint32_t custom_mode = 0;

  uint16_t voltage_battery_mV = 0;
  int16_t current_battery_cA = 0;
  int8_t battery_remaining = -1;
  uint16_t drop_rate_comm = 0;

  float roll_deg = 0;
  float pitch_deg = 0;
  float yaw_deg = 0;

  double lat_deg = 0;
  double lon_deg = 0;
  float alt_m = 0;
  float rel_alt_m = 0;
  float vx_ms = 0;
  float vy_ms = 0;
  float vz_ms = 0;
  float heading_deg = NAN;

  uint8_t gps_fix_type = 0;
  uint8_t satellites_visible = 0;
  uint16_t gps_hdop = UINT16_MAX;

  float airspeed = 0;
  float groundspeed = 0;
  int16_t heading_hud = 0;
  uint16_t throttle = 0;
  float alt_hud = 0;
  float climb = 0;

  float battery_voltage_V = 0;
  float battery_current_A = NAN;
  int32_t battery_remaining_pct = -1;

  uint32_t last_heartbeat_ms = 0;
};

static HardwareSerial& mavSerial = Serial1;
static Telemetry telem;
static bool streams_requested = false;
static uint32_t last_heartbeat_sent_ms = 0;
static uint32_t last_print_ms = 0;
static uint32_t last_stream_request_ms = 0;

static const char* autopilotName(uint8_t autopilot) {
  switch (autopilot) {
    case MAV_AUTOPILOT_ARDUPILOTMEGA:
      return "ArduPilot";
    case MAV_AUTOPILOT_PX4:
      return "PX4";
    case MAV_AUTOPILOT_GENERIC:
      return "Generic";
    default:
      return "Other";
  }
}

static const char* systemStateName(uint8_t state) {
  switch (state) {
    case MAV_STATE_UNINIT:
      return "UNINIT";
    case MAV_STATE_BOOT:
      return "BOOT";
    case MAV_STATE_CALIBRATING:
      return "CALIBRATING";
    case MAV_STATE_STANDBY:
      return "STANDBY";
    case MAV_STATE_ACTIVE:
      return "ACTIVE";
    case MAV_STATE_CRITICAL:
      return "CRITICAL";
    case MAV_STATE_EMERGENCY:
      return "EMERGENCY";
    case MAV_STATE_POWEROFF:
      return "POWEROFF";
    case MAV_STATE_FLIGHT_TERMINATION:
      return "FLIGHT_TERMINATION";
    default:
      return "UNKNOWN";
  }
}

static const char* gpsFixName(uint8_t fix) {
  switch (fix) {
    case GPS_FIX_TYPE_NO_GPS:
      return "NO_GPS";
    case GPS_FIX_TYPE_NO_FIX:
      return "NO_FIX";
    case GPS_FIX_TYPE_2D_FIX:
      return "2D";
    case GPS_FIX_TYPE_3D_FIX:
      return "3D";
    case GPS_FIX_TYPE_DGPS:
      return "DGPS";
    case GPS_FIX_TYPE_RTK_FLOAT:
      return "RTK_FLOAT";
    case GPS_FIX_TYPE_RTK_FIXED:
      return "RTK_FIXED";
    default:
      return "OTHER";
  }
}

static void sendMavlink(const mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);
}

static void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(GCS_SYS_ID, GCS_COMP_ID, &msg, MAV_TYPE_GCS,
                             MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  sendMavlink(msg);
}

static void requestMessageInterval(uint32_t msgid) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      GCS_SYS_ID, GCS_COMP_ID, &msg, telem.sysid, telem.compid,
      MAV_CMD_SET_MESSAGE_INTERVAL, 0, static_cast<float>(msgid),
      static_cast<float>(MESSAGE_INTERVAL_US), 0, 0, 0, 0, 0);
  sendMavlink(msg);
}

static void requestLegacyDataStream(uint8_t stream_id) {
  mavlink_message_t msg;
  mavlink_msg_request_data_stream_pack(GCS_SYS_ID, GCS_COMP_ID, &msg, telem.sysid,
                                       telem.compid, stream_id, 1, 1);
  sendMavlink(msg);
}

static void requestTelemetryStreams() {
  if (telem.sysid == 0) {
    return;
  }

  requestMessageInterval(MAVLINK_MSG_ID_SYS_STATUS);
  requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE);
  requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
  requestMessageInterval(MAVLINK_MSG_ID_GPS_RAW_INT);
  requestMessageInterval(MAVLINK_MSG_ID_VFR_HUD);
  requestMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS);

  // Fallback for older ArduPilot versions that ignore SET_MESSAGE_INTERVAL.
  requestLegacyDataStream(MAV_DATA_STREAM_EXTENDED_STATUS);
  requestLegacyDataStream(MAV_DATA_STREAM_POSITION);
  requestLegacyDataStream(MAV_DATA_STREAM_EXTRA1);
  requestLegacyDataStream(MAV_DATA_STREAM_EXTRA2);

  streams_requested = true;
  last_stream_request_ms = millis();
  Serial.printf("Requested 1 Hz telemetry from sys %u comp %u\n", telem.sysid,
                telem.compid);
}

static void handleHeartbeat(const mavlink_message_t& msg) {
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(&msg, &hb);

  if (hb.autopilot == MAV_AUTOPILOT_INVALID) {
    return;
  }

  const bool first = !telem.has_heartbeat;
  telem.has_heartbeat = true;
  telem.sysid = msg.sysid;
  telem.compid = msg.compid;
  telem.type = hb.type;
  telem.autopilot = hb.autopilot;
  telem.base_mode = hb.base_mode;
  telem.system_status = hb.system_status;
  telem.custom_mode = hb.custom_mode;
  telem.last_heartbeat_ms = millis();

  if (first) {
    Serial.printf("FC heartbeat: sys=%u comp=%u autopilot=%s armed=%s state=%s\n",
                  telem.sysid, telem.compid, autopilotName(telem.autopilot),
                  (telem.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "YES" : "NO",
                  systemStateName(telem.system_status));
    requestTelemetryStreams();
  }
}

static void handleSysStatus(const mavlink_message_t& msg) {
  mavlink_sys_status_t status;
  mavlink_msg_sys_status_decode(&msg, &status);
  telem.has_sys_status = true;
  telem.voltage_battery_mV = status.voltage_battery;
  telem.current_battery_cA = status.current_battery;
  telem.battery_remaining = status.battery_remaining;
  telem.drop_rate_comm = status.drop_rate_comm;
}

static void handleAttitude(const mavlink_message_t& msg) {
  mavlink_attitude_t att;
  mavlink_msg_attitude_decode(&msg, &att);
  telem.has_attitude = true;
  telem.roll_deg = att.roll * 180.0f / PI;
  telem.pitch_deg = att.pitch * 180.0f / PI;
  telem.yaw_deg = att.yaw * 180.0f / PI;
}

static void handleGlobalPosition(const mavlink_message_t& msg) {
  mavlink_global_position_int_t pos;
  mavlink_msg_global_position_int_decode(&msg, &pos);
  telem.has_position = true;
  telem.lat_deg = pos.lat / 1e7;
  telem.lon_deg = pos.lon / 1e7;
  telem.alt_m = pos.alt / 1000.0f;
  telem.rel_alt_m = pos.relative_alt / 1000.0f;
  telem.vx_ms = pos.vx / 100.0f;
  telem.vy_ms = pos.vy / 100.0f;
  telem.vz_ms = pos.vz / 100.0f;
  telem.heading_deg = (pos.hdg == UINT16_MAX) ? NAN : (pos.hdg / 100.0f);
}

static void handleGpsRaw(const mavlink_message_t& msg) {
  mavlink_gps_raw_int_t gps;
  mavlink_msg_gps_raw_int_decode(&msg, &gps);
  telem.has_gps = true;
  telem.gps_fix_type = gps.fix_type;
  telem.satellites_visible = gps.satellites_visible;
  telem.gps_hdop = gps.eph;
}

static void handleVfrHud(const mavlink_message_t& msg) {
  mavlink_vfr_hud_t hud;
  mavlink_msg_vfr_hud_decode(&msg, &hud);
  telem.has_vfr = true;
  telem.airspeed = hud.airspeed;
  telem.groundspeed = hud.groundspeed;
  telem.heading_hud = hud.heading;
  telem.throttle = hud.throttle;
  telem.alt_hud = hud.alt;
  telem.climb = hud.climb;
}

static void handleBattery(const mavlink_message_t& msg) {
  mavlink_battery_status_t bat;
  mavlink_msg_battery_status_decode(&msg, &bat);
  telem.has_battery = true;
  if (bat.voltages[0] != UINT16_MAX) {
    telem.battery_voltage_V = bat.voltages[0] / 1000.0f;
  }
  telem.battery_current_A =
      (bat.current_battery < 0) ? NAN : (bat.current_battery / 100.0f);
  telem.battery_remaining_pct = bat.battery_remaining;
}

static void handleStatusText(const mavlink_message_t& msg) {
  mavlink_statustext_t text;
  mavlink_msg_statustext_decode(&msg, &text);
  char line[51];
  memcpy(line, text.text, 50);
  line[50] = '\0';
  Serial.printf("[FC STATUSTEXT sev=%u] %s\n", text.severity, line);
}

static void handleMavlinkMessage(const mavlink_message_t& msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      handleHeartbeat(msg);
      break;
    case MAVLINK_MSG_ID_SYS_STATUS:
      handleSysStatus(msg);
      break;
    case MAVLINK_MSG_ID_ATTITUDE:
      handleAttitude(msg);
      break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      handleGlobalPosition(msg);
      break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      handleGpsRaw(msg);
      break;
    case MAVLINK_MSG_ID_VFR_HUD:
      handleVfrHud(msg);
      break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
      handleBattery(msg);
      break;
    case MAVLINK_MSG_ID_STATUSTEXT:
      handleStatusText(msg);
      break;
    default:
      break;
  }
}

static void readMavlink() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (mavSerial.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(mavSerial.read());
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
      handleMavlinkMessage(msg);
    }
  }
}

static void printTelemetry() {
  Serial.println("---------- MAVLink telemetry (1 Hz) ----------");

  if (!telem.has_heartbeat) {
    Serial.println("Waiting for flight controller heartbeat...");
    Serial.println("Check UART wiring, baud rate, and shared GND.");
    return;
  }

  const bool stale = (millis() - telem.last_heartbeat_ms) > FC_TIMEOUT_MS;
  Serial.printf("FC: sys=%u comp=%u %s  armed=%s  state=%s%s\n", telem.sysid,
                telem.compid, autopilotName(telem.autopilot),
                (telem.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "YES" : "NO",
                systemStateName(telem.system_status),
                stale ? "  [STALE]" : "");
  Serial.printf("Mode flags=0x%02X  custom_mode=%lu\n", telem.base_mode,
                static_cast<unsigned long>(telem.custom_mode));

  if (telem.has_attitude) {
    Serial.printf("Attitude: roll=%.1f deg  pitch=%.1f deg  yaw=%.1f deg\n",
                  telem.roll_deg, telem.pitch_deg, telem.yaw_deg);
  }

  if (telem.has_position) {
    Serial.printf("Position: lat=%.7f  lon=%.7f  alt=%.1f m  rel=%.1f m\n",
                  telem.lat_deg, telem.lon_deg, telem.alt_m, telem.rel_alt_m);
    if (!isnan(telem.heading_deg)) {
      Serial.printf("Velocity: vn=%.2f ve=%.2f vd=%.2f m/s  hdg=%.1f deg\n",
                    telem.vx_ms, telem.vy_ms, telem.vz_ms, telem.heading_deg);
    }
  }

  if (telem.has_gps) {
    Serial.printf("GPS: fix=%s  sats=%u  hdop=%.2f\n",
                  gpsFixName(telem.gps_fix_type), telem.satellites_visible,
                  telem.gps_hdop == UINT16_MAX ? NAN : (telem.gps_hdop / 100.0f));
  }

  if (telem.has_vfr) {
    Serial.printf("HUD: AS=%.1f  GS=%.1f m/s  hdg=%d  thr=%u%%  alt=%.1f  climb=%.2f\n",
                  telem.airspeed, telem.groundspeed, telem.heading_hud,
                  telem.throttle, telem.alt_hud, telem.climb);
  }

  if (telem.has_sys_status) {
    Serial.printf("SYS batt: %.2f V  %.2f A  remain=%d%%  drop=%u\n",
                  telem.voltage_battery_mV / 1000.0f,
                  telem.current_battery_cA / 100.0f, telem.battery_remaining,
                  telem.drop_rate_comm);
  }

  if (telem.has_battery) {
    Serial.printf("Battery: %.2f V", telem.battery_voltage_V);
    if (!isnan(telem.battery_current_A)) {
      Serial.printf("  %.2f A", telem.battery_current_A);
    }
    Serial.printf("  remain=%ld%%\n",
                  static_cast<long>(telem.battery_remaining_pct));
  }
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(1500);

  mavSerial.setRxBufferSize(1024);
  mavSerial.begin(MAV_BAUD, SERIAL_8N1, MAV_RX_PIN, MAV_TX_PIN);

  Serial.println();
  Serial.println("ESP32-S3 MAVLink2 telemetry bridge");
  Serial.printf("FC UART: RX=%d TX=%d baud=%lu\n", MAV_RX_PIN, MAV_TX_PIN,
                static_cast<unsigned long>(MAV_BAUD));
  Serial.println("Debug output: USB Serial @ 115200");
  Serial.println("Waiting for flight controller...");

  sendHeartbeat();
  last_heartbeat_sent_ms = millis();
}

void loop() {
  const uint32_t now = millis();

  readMavlink();

  if (now - last_heartbeat_sent_ms >= HEARTBEAT_PERIOD_MS) {
    sendHeartbeat();
    last_heartbeat_sent_ms = now;
  }

  if (streams_requested &&
      (now - last_stream_request_ms >= STREAM_REFRESH_PERIOD_MS)) {
    requestTelemetryStreams();
  }

  if (now - last_print_ms >= TELEMETRY_PRINT_PERIOD_MS) {
    printTelemetry();
    last_print_ms = now;
  }

  // wifiManager.loop();
  
}
