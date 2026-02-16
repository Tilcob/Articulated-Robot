#include "SerialControl.h"
#include "Util.h"

#include <Arduino.h>
#include <ctype.h>
#include <HardwareSerial.h>
#include <stdlib.h>

static void skipSpaces(char*& p) {
  while (*p && isspace(static_cast<unsigned char>(*p))) p++;
}

bool readSerialTcpCommand(bool& debugPrint,
                          const bool havePose,
                          const Vec3& lastTcpPosBase,
                          const Mat3& lastRBaseTcp,
                          Vec3& outPos,
                          float& outGrip01) {
  static char buf[80];
  static uint8_t len = 0;

  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      buf[len] = '\0';
      len = 0;

      char* p = buf;
      skipSpaces(p);

      if (*p == 'h' || *p == 'H') {
        Serial.println("Commands:");
        Serial.println("  x y z [g]      -> absolute Base-KS");
        Serial.println("  t dx dy dz [g] -> relative TCP-KS");
        Serial.println("  d              -> toggle debug");
        return false;
      }

      if (*p == 'd' || *p == 'D') {
        debugPrint = !debugPrint;
        Serial.print("DEBUG: ");
        Serial.println(debugPrint ? "ON" : "OFF");
        return false;
      }

      bool tcpRelative = false;
      if (*p == 't' || *p == 'T') {
        tcpRelative = true;
        p++;
        skipSpaces(p);
      }

      char* end;
      const double xd = strtod(p, &end); if (end == p) return false; p = end;
      const double yd = strtod(p, &end); if (end == p) return false; p = end;
      const double zd = strtod(p, &end); if (end == p) return false; p = end;

      double gd = outGrip01;
      skipSpaces(p);
      if (*p != '\0') gd = strtod(p, &end);

      const Vec3 parsed{static_cast<float>(xd), static_cast<float>(yd), static_cast<float>(zd)};
      const float g = constrain((float)gd, 0.0f, 1.0f);
      outGrip01 = g;

      if (tcpRelative) {
        if (!havePose) {
          Serial.println("ERR: no pose yet");
          return false;
        }
        const Vec3 d_base = math3::mul(lastRBaseTcp, parsed);
        outPos = math3::add(lastTcpPosBase, d_base);
      } else {
        outPos = parsed;
      }
      return true;
    }

    if (len < sizeof(buf) - 1) buf[len++] = c;
    else len = 0;
  }
  return false;
}
