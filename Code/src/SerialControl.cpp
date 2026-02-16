#include "SerialControl.h"

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

#include "Util.h"

void SerialControl::printHelp() {
  Serial.println("Commands:");
  Serial.println("  h                -> help");
  Serial.println("  d0 / d1          -> debug off/on");
  Serial.println("  x y z            -> absolute TCP target (base frame)");
  Serial.println("  x y z g          -> absolute TCP + gripper (0..1)");
  Serial.println("  t dx dy dz       -> TCP-relative move in TCP frame (needs pose)");
  Serial.println("  t dx dy dz g     -> TCP-relative + gripper (0..1)");
}

void SerialControl::skipSpaces_(char*& p) {
  while (*p && isspace(static_cast<unsigned char>(*p))) p++;
}

bool SerialControl::poll(bool& debugPrint,
                         const bool havePose,
                         const Vec3& lastTcpPosBase,
                         const Mat3& lastRBaseTcp,
                         Vec3& outPos,
                         float& outGrip01) {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      buf_[len_] = '\0';
      len_ = 0;
      return parseLine_(buf_, debugPrint, havePose, lastTcpPosBase, lastRBaseTcp, outPos, outGrip01);
    }

    if (len_ < sizeof(buf_) - 1) {
      buf_[len_++] = c;
    } else {
      // overflow -> reset line
      len_ = 0;
    }
  }
  return false;
}

bool SerialControl::parseLine_(char* line,
                               bool& debugPrint,
                               const bool havePose,
                               const Vec3& lastTcpPosBase,
                               const Mat3& lastRBaseTcp,
                               Vec3& outPos,
                               float& outGrip01) {
  char* p = line;
  skipSpaces_(p);
  if (*p == '\0') return false;

  // help
  if (*p == 'h' || *p == 'H' || *p == '?') {
    printHelp();
    return false;
  }

  // debug toggle: d0 / d1
  if (*p == 'd' || *p == 'D') {
    p++;
    skipSpaces_(p);
    if (*p == '0') { debugPrint = false; Serial.println("DBG off"); }
    else if (*p == '1') { debugPrint = true; Serial.println("DBG on"); }
    else Serial.println("ERR: use d0 or d1");
    return false;
  }

  bool tcpRelative = false;
  if (*p == 't' || *p == 'T') {
    tcpRelative = true;
    p++;
    skipSpaces_(p);
  }

  // parse x y z [g]
  char* end = nullptr;

  const double xd = strtod(p, &end);
  if (end == p) { Serial.println("ERR: parse"); return false; }
  p = end; skipSpaces_(p);

  const double yd = strtod(p, &end);
  if (end == p) { Serial.println("ERR: parse"); return false; }
  p = end; skipSpaces_(p);

  const double zd = strtod(p, &end);
  if (end == p) { Serial.println("ERR: parse"); return false; }
  p = end; skipSpaces_(p);

  double gd = outGrip01; // default: keep current
  if (*p != '\0') {
    gd = strtod(p, &end);
    // if parse fails, keep the old grip
    if (end == p) gd = outGrip01;
  }

  const Vec3 parsed{static_cast<float>(xd), static_cast<float>(yd), static_cast<float>(zd)};
  const float g = util::clamp01(static_cast<float>(gd));
  outGrip01 = g;

  if (tcpRelative) {
    if (!havePose) {
      Serial.println("ERR: no pose yet");
      return false;
    }
    // parsed is in TCP frame -> convert to base: d_base = R_base_tcp * d_tcp
    const Vec3 dBase = math3::mul(lastRBaseTcp, parsed);
    outPos = math3::add(lastTcpPosBase, dBase);
  } else {
    outPos = parsed;
  }

  return true;
}
