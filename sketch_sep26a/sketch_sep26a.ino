#include <Servo.h>
Servo myservo;

// ==================== Pins ====================
#define TRIG 12
#define ECHO 13
#define ENA  5
#define ENB  6
#define IN1  3
#define IN2  4
#define IN3  2
#define IN4  7

// ==================== Servo ====================
#define SERVO_MIN    10
#define SERVO_MAX    170
#define SCAN_STEP    20
#define SN_MAX       9          // (170-10)/20+1 = 9 samples
#define SERVO_DWELL  60         // ms between samples (moving scan)

// ==================== Polar-sector histogram (VFH) ====================
#define SECTOR_W     15          // sector width (deg)
#define HALF_SPAN    90          // cover +-90
#define SECTOR_N     12          // (2*90)/15
#define RANGE_MIN    3           // ignore too-close unreliable reads
#define RANGE_MAX    100         // beyond this => no obstacle
#define OCC_MAX      15          // occupancy count saturation

// ==================== Body width & passable-channel ====================
#define CAR_WIDTH    15          // car body width cm
#define CLEAR_MARGIN 4           // extra side clearance cm
#define PASS_SECTORS 3           // contiguous open sectors for a passable channel
                                 // (center + 1 each side => ~45deg window)

// ==================== Control params ====================
#define WALL_STOP    16          // emergency-stop guard distance cm
#define TURN_DIST    32          // near => stop and turn
#define SLOW_DIST    48          // medium => decelerate
#define HYST         6           // decel->forward hysteresis band
#define DEAD_ZONE    15          // turn dead-zone (deg from straight)
#define FREESPACE    20          // verify distance after turn cm
#define MAXTURN      2           // consecutive turn attempts before backup
#define BACKUP_MS    450         // reverse-out duration ms
#define TURN_MS      350         // open-loop turn duration ms
#define SPD_HIGH     130
#define SPD_LOW      80
#define SPD_TURN     200

// ==================== Contour match (closed-loop heading) ====================
#define MATCH_SCAN_MS       40    // parked static-sweep dwell per sample (ms)
#define MATCH_MIN_PAIRS     5     // min valid overlapping pairs (else unreliable)
#define MATCH_MAX_PAIR_DIFF 3     // max avg per-pair diff cm (reliability b)
#define MATCH_MARGIN        2     // best vs second-best err gap (peak sharpness)

// ==================== Disengage / stuck watchdog ====================
#define DISENGAGE_MS        600   // disengage reverse duration ms
#define DISENGAGE_MAX       3     // disengage trigger cap, beyond => double reverse
#define STUCK_TIMEOUT       2500  // ms without forward progress => force disengage
#define STUCK_MAX           8     // per-sector stuck-count saturation
#define HALT_TIMEOUT        1500  // ms dead-stopped at wall before forced reverse
#define PROGRESS_DELTA      4     // cm of nearest-obstacle change => real displacement

// ==================== States ====================
#define ST_SCAN      1
#define ST_FORWARD   2
#define ST_DECEL     3
#define ST_TURN      4
#define ST_VERIFY    5
#define ST_BACKUP    6
#define ST_HALT      7
#define ST_CAPREF    8
#define ST_CAPNEW    9
#define ST_DISENGAGE 10

// ==================== Sector data ====================
typedef struct {
  uint8_t dist;    // latest measured distance in this sector (cm), 0 = invalid
  uint8_t count;   // accumulated confirmed-obstacle count (0..OCC_MAX)
} Sector;

Sector sector[SECTOR_N];          // 24 B  polar sector histogram
uint8_t secOffset = 0;            // 1 B  heading offset on the sector ring
uint8_t state      = ST_SCAN;
uint8_t scanIdx    = 0;
uint8_t turnCount  = 0;
uint8_t planHeadIdx = 0;
int8_t  turnStep   = 0;           // signed sectors to turn (-6..+6)
unsigned long lastScanMs = 0;
unsigned long moveStartMs = 0;

// ==================== Closed-loop calibration globals ====================
uint8_t refContour[SECTOR_N];     // 12 B  parked reference contour (physical sectors)
uint8_t newContour[SECTOR_N];     // 12 B  parked post-turn contour
uint8_t secOffsetOld = 0;         // 1  B  secOffset before turn (calibration base)
int8_t  bestShift    = 0;         // 1  B  best shift = measured true rotation (-6..+6)
uint16_t bestScore   = 0;         // 2  B  best-shift error sum
int8_t  matchPairs   = 0;         // 1  B  valid overlapping pairs at best shift
uint8_t matchReliable = 0;        // 1  B  0=unreliable 1=reliable
uint8_t capScanIdx   = 0;         // 1  B  parked static-sweep sample index
uint8_t disengCount  = 0;         // 1  B  disengage trigger count
unsigned long disengUntilMs = 0;  // 4  B  disengage end timestamp

// ==================== Iterative stuck-learning globals ====================
uint8_t stuck[SECTOR_N];          // 12 B  decayed per-sector stuck counter
unsigned long lastProgressMs = 0; // 4  B  last time real forward progress happened
unsigned long haltStartMs = 0;    // 4  B  when the wall-halt began (HALT_TIMEOUT base)
uint8_t lastProgNearest = 0;      // 1  B  nearest-obstacle distance at last progress event

// ==================== Motor drivers ====================
void motorStop(void) {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void motorForward(byte spd) {
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void motorBack(void) {
  analogWrite(ENA, SPD_LOW);
  analogWrite(ENB, SPD_LOW);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

// dir>0 => turn left (toward +), dir<0 => turn right
void motorTurn(int8_t dir) {
  analogWrite(ENA, SPD_TURN);
  analogWrite(ENB, SPD_TURN);
  if (dir > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }
}

// ==================== Ultrasonic (returns cm, 0 = invalid) ====================
int GetDistance(void) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH, 30000);
  if (t == 0) return 0;
  return (int)(t / 58L);
}

// ==================== Sector helpers ====================
// round(deg / SECTOR_W), works for negatives (round half away from zero)
int8_t degToStep(int16_t deg) {
  if (deg >= 0) return (int8_t)((deg + SECTOR_W / 2) / SECTOR_W);
  else          return (int8_t)(-((-deg + SECTOR_W / 2) / SECTOR_W));
}

// physical sector index for a relative angle rel (deg, -90..+90)
uint8_t relToSector(int8_t rel) {
  int8_t step = degToStep(rel);
  return (uint8_t)((secOffset + step + SECTOR_N) % SECTOR_N);
}

// apply a turn of deltaDeg: only move heading offset, never clear data
void applyTurn(int16_t deltaDeg) {
  int8_t step = degToStep(deltaDeg);
  secOffset = (uint8_t)((secOffset + step + SECTOR_N) % SECTOR_N);
}

// center angle of sector idx relative to heading, signed -90..+90
int16_t sectorCenter(int8_t idx) {
  if (idx <= 6) return (int16_t)(idx * SECTOR_W);
  return (int16_t)((idx - SECTOR_N) * SECTOR_W);
}

// shortest signed step from a to b on the ring (-6..+6)
int8_t ringStep(uint8_t a, uint8_t b) {
  int16_t d = (int16_t)b - (int16_t)a;
  if (d >  SECTOR_N / 2) d -= SECTOR_N;
  if (d < -SECTOR_N / 2) d += SECTOR_N;
  return (int8_t)d;
}

// write a fresh reading at relative angle rel into its physical sector
void sectorWrite(int8_t rel, int d) {
  uint8_t idx = relToSector(rel);
  if (d >= RANGE_MIN && d <= RANGE_MAX) sector[idx].dist = (uint8_t)d;
  else sector[idx].dist = 0;
  if (sector[idx].count < OCC_MAX) sector[idx].count++;
}

void mapDecay(void) {
  for (int i = 0; i < SECTOR_N; i++) {
    uint8_t c = sector[i].count;
    if (c > 1) sector[i].count = (uint8_t)(c >> 1);   // half-life decay
  }
}

// ==================== Passable channel (body-width aware) ====================
// A direction idx is passable only if the whole channel centered on it is open:
// every sector in the +-(PASS_SECTORS-1)/2 window reads >= CHANNEL_DIST.
// This makes the car refuse gaps narrower than its own body.
uint8_t passable(uint8_t idx) {
  int8_t half = (int8_t)((PASS_SECTORS - 1) / 2);
  for (int8_t k = -half; k <= half; k++) {
    uint8_t s = (uint8_t)((idx + k + SECTOR_N) % SECTOR_N);
    if (sector[s].dist < TURN_DIST) return 0;   // includes blind(0)/near obstacle
  }
  return 1;
}

// ==================== Iterative stuck-learning ====================
void markStuck(uint8_t idx) {
  for (int8_t k = -1; k <= 1; k++) {
    uint8_t s = (uint8_t)((idx + k + SECTOR_N) % SECTOR_N);
    if (stuck[s] < STUCK_MAX) stuck[s]++;
  }
}

void stuckDecay(void) {
  for (int i = 0; i < SECTOR_N; i++) {
    if (stuck[i] > 0) stuck[i]--;
  }
}

// nearest valid obstacle distance across all sectors (0/blind treated as open);
// returns RANGE_MAX when nothing within range (fully open road)
uint8_t nearestDist(void) {
  uint8_t m = RANGE_MAX;
  for (int i = 0; i < SECTOR_N; i++) {
    uint8_t d = sector[i].dist;
    if (d != 0 && d < m) m = d;
  }
  return m;
}

// refresh lastProgressMs ONLY on genuine forward progress: either the nearest
// obstacle materially changed (real displacement) or the road is fully open.
// A car wedged sideways (wheels spinning, front open) keeps nearest constant and
// therefore does NOT refresh, so the STUCK_TIMEOUT watchdog fires and reverses it.
void refreshProgress(void) {
  uint8_t nn = nearestDist();
  int16_t delta = (int16_t)nn - (int16_t)lastProgNearest;
  if (delta < 0) delta = -delta;
  if (nn == RANGE_MAX || delta >= PROGRESS_DELTA) {
    lastProgressMs = millis();
    lastProgNearest = nn;
  }
}

// ==================== Moving scan (real-time guard only, never contour) ====================
void scanTick(void) {
  unsigned long now = millis();
  if (now - lastScanMs < SERVO_DWELL) return;
  lastScanMs = now;

  int theta = SERVO_MIN + scanIdx * SCAN_STEP;   // current settled angle
  int d = GetDistance();
  sectorWrite((int8_t)(theta - 90), d);          // rel = theta - 90

  scanIdx++;
  if (scanIdx >= SN_MAX) { scanIdx = 0; mapDecay(); stuckDecay(); }
  myservo.write(SERVO_MIN + scanIdx * SCAN_STEP);
}

// ==================== Contour match: estimate real rotation ====================
// Full-integer. Slides shift s in 0..11, keeps best + second-best peaks.
void matchContour(const uint8_t* R, const uint8_t* C) {
  int8_t  bestS = 0, secondS = -1;
  uint16_t bestE = 0xFFFF, secondE = 0xFFFF;
  int8_t  bestP = 0;

  for (int8_t s = 0; s < SECTOR_N; s++) {
    uint16_t err = 0;
    int8_t   pairs = 0;
    for (int8_t i = 0; i < SECTOR_N; i++) {
      uint8_t r = R[(uint8_t)((i - s + SECTOR_N) % SECTOR_N)];
      uint8_t c = C[i];
      if (r == 0 || c == 0) continue;                 // skip empty/blind/open pairs
      int16_t d = (int16_t)r - (int16_t)c;
      if (d < 0) d = -d;
      pairs++;
      err += (uint16_t)d;
    }
    if (pairs == 0) continue;
    if (err < bestE || (err == bestE && pairs > bestP)) {
      secondE = bestE; secondS = bestS;               // keep top-2 peaks
      bestE = err; bestS = s; bestP = pairs;
    } else if (err < secondE) {
      secondE = err; secondS = s;
    }
  }

  bestShift = bestS;
  if (bestShift > SECTOR_N / 2) bestShift = (int8_t)(bestShift - SECTOR_N);
  bestScore  = bestE;
  matchPairs = bestP;

  matchReliable =
      (bestP >= MATCH_MIN_PAIRS)
   && (bestE <= (uint16_t)(bestP * MATCH_MAX_PAIR_DIFF))
   && (secondS != -1 && (int16_t)secondE - (int16_t)bestE >= MATCH_MARGIN);
}

// ==================== Closed-loop calibration ====================
// Rebuild secOffset from measured bestShift (anti-drift), or fall back to open loop.
void applyCalibration(void) {
  if (matchReliable) {
    secOffset = (uint8_t)((secOffsetOld + bestShift + SECTOR_N) % SECTOR_N);
  } else {
    secOffset = (uint8_t)((secOffsetOld + turnStep + SECTOR_N) % SECTOR_N);
  }
}

// ==================== Turn planning (passable-channel + stuck aware) ====================
void planTurn(void) {
  uint8_t front = sector[secOffset].dist;
  planHeadIdx = secOffset;
  // B1: go straight only if front is known-open AND the channel is wide enough
  if (front >= TURN_DIST && passable(secOffset)) { planHeadIdx = secOffset; return; }
  int8_t best = -1;
  uint16_t bestScore = 0xFFFF;
  uint8_t bdir = secOffset;
  for (int8_t i = 1; i < SECTOR_N; i++) {
    uint8_t idx = (uint8_t)((secOffset + i) % SECTOR_N);
    if (!passable(idx)) continue;         // channel too narrow / blind / near obstacle
    if (stuck[idx] > 0) continue;         // B4: learned: this direction got us stuck
    uint8_t d = sector[idx].dist;
    if (d < TURN_DIST) continue;          // not passable distance
    int16_t off = sectorCenter(i);
    if (off < 0) off = -off;              // absolute offset from front
    uint16_t score = (uint16_t)d * 2 - (uint16_t)off * 1;   // dist wt 2, off wt 1
    if (score < bestScore) { bestScore = score; best = idx; bdir = idx; }
  }
  if (best < 0) { enterDisengage(); return; }   // B2: all blocked => disengage
  planHeadIdx = bdir;
}

// ==================== Closed-loop turn sequence ====================
void enterCapRef(void) {
  motorStop();
  secOffsetOld = secOffset;         // snapshot heading before this turn sequence
  capScanIdx = 0;
  state = ST_CAPREF;
}

// one step of the parked static sweep into refContour
void capRefTick(void) {
  if (capScanIdx < SN_MAX) {
    int theta = SERVO_MIN + capScanIdx * SCAN_STEP;
    myservo.write(theta);
    delay(MATCH_SCAN_MS);
    int d = GetDistance();
    int8_t rel = (int8_t)(theta - 90);
    refContour[relToSector(rel)] = (d >= RANGE_MIN && d <= RANGE_MAX) ? (uint8_t)d : 0;
    capScanIdx++;
  } else {
    // scan done: plan turn on clean data
    planTurn();
    turnStep = ringStep(secOffsetOld, planHeadIdx);
    if (turnStep == 0) { turnCount = 0; state = ST_FORWARD; motorForward(SPD_HIGH); refreshProgress(); }
    else { turnCount++; enterTurn(); }          // command motor, go TURN
  }
}

void enterTurn(void) {
  if (turnStep == 0) { motorForward(SPD_HIGH); return; }
  motorTurn(turnStep > 0 ? 1 : -1);
  moveStartMs = millis();
  state = ST_TURN;
}

void capNewTick(void) {
  if (capScanIdx < SN_MAX) {
    int theta = SERVO_MIN + capScanIdx * SCAN_STEP;
    myservo.write(theta);
    delay(MATCH_SCAN_MS);
    int d = GetDistance();
    int8_t rel = (int8_t)(theta - 90);
    newContour[relToSector(rel)] = (d >= RANGE_MIN && d <= RANGE_MAX) ? (uint8_t)d : 0;
    capScanIdx++;
  } else {
    // scan done: match reference vs new contour
    matchContour(refContour, newContour);
    if (turnStep != 0 && matchReliable && bestShift == 0) {
      enterDisengage();                       // B3: physical stuck, body did not rotate
      return;
    }
    applyCalibration();                       // closed-loop heading correction
    state = ST_VERIFY;
  }
}

// ==================== Disengage (stronger than BACKUP) ====================
void enterDisengage(void) {
  markStuck(planHeadIdx);               // remember the direction that failed
  disengCount++;
  unsigned long dMs = DISENGAGE_MS;
  if (disengCount >= DISENGAGE_MAX) dMs *= 2;   // escalate reverse duration
  moveStartMs = millis();
  disengUntilMs = moveStartMs + dMs;
  motorStop();
  state = ST_DISENGAGE;
}

void disengTick(void) {
  if ((long)(millis() - disengUntilMs) >= 0) {   // elapsed past absolute end (overflow-safe)
    motorStop();
    turnCount = 0;
    enterCapRef();
  } else {
    motorBack();                              // reverse only, never turn while stuck
  }
}

// ==================== Verify after turn (closed-loop sector feedback) ====================
void verifyAfterTurn(void) {
  uint8_t front = sector[secOffset].dist;
  if (front >= FREESPACE) {
    turnCount = 0; disengCount = 0;   // cleared the obstacle: reset disengage streak
    lastProgressMs = millis();        // real forward event
    lastProgNearest = nearestDist();
    state = ST_FORWARD;
    return;
  }
  if (turnCount >= MAXTURN) {
    enterDisengage();                 // repeated turns gave no clearance => force reverse escape
    return;
  }
  enterCapRef();                      // refresh refContour + secOffsetOld, re-plan turn
}

// ==================== Loop helpers ====================
int turnDone(void) { return (millis() - moveStartMs) >= TURN_MS; }

// ==================== Setup ====================
void setup(void) {
  myservo.attach(A0, 700, 2400);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  motorStop();
  for (int i = 0; i < SECTOR_N; i++) { sector[i].dist = 0; sector[i].count = 0; }
  for (int i = 0; i < SECTOR_N; i++) { refContour[i] = 0; newContour[i] = 0; }
  for (int i = 0; i < SECTOR_N; i++) { stuck[i] = 0; }
  scanIdx = 0;
  myservo.write(SERVO_MIN);
  lastScanMs = millis();
  lastProgressMs = millis();
  lastProgNearest = nearestDist();
}

// ==================== Main loop ====================
void loop(void) {
  scanTick();                     // keep servo scanning & sectors fresh (guard only)

  uint8_t front = sector[secOffset].dist;

  // global guard: too close ahead => immediate stop (with HALT_TIMEOUT escape).
  // During a deliberate reverse-escape (ST_DISENGAGE) do NOT re-halt, let it back out.
  if (front > 0 && front <= WALL_STOP) {
    if (state != ST_HALT && state != ST_DISENGAGE) {
      motorStop(); haltStartMs = millis(); state = ST_HALT;
    }
  } else if (state == ST_HALT) {
    state = ST_FORWARD;              // obstacle cleared; resume (real forward event)
    lastProgressMs = millis();
    lastProgNearest = nearestDist();
  }

  // C: no real forward progress for too long in any non-reversing state => reverse
  if ((unsigned long)(millis() - lastProgressMs) > STUCK_TIMEOUT) {
    if (state != ST_DISENGAGE && state != ST_HALT) {
      enterDisengage();             // unconditional forced reverse (car stuck)
    }
  }
  // R1-1: dead-stopped at a wall longer than HALT_TIMEOUT => forced reverse escape
  if (state == ST_HALT &&
      (unsigned long)(millis() - haltStartMs) > HALT_TIMEOUT) {
    enterDisengage();
  }

  switch (state) {
    case ST_SCAN:
    case ST_FORWARD:
      if (front <= SLOW_DIST) state = ST_DECEL;
      else {
        motorForward(SPD_HIGH);
        disengCount = 0;            // normal advance => reset disengage streak
        refreshProgress();          // real forward event (open road or displacement)
      }
      break;

    case ST_DECEL:
      if (front > SLOW_DIST + HYST) { state = ST_FORWARD; refreshProgress(); }
      else if (front > 0 && front <= TURN_DIST) {         // B1: only turn on real near obstacle
        enterCapRef();
      }
      else { motorForward(SPD_LOW); refreshProgress(); }  // slow approach is real progress
      break;

    case ST_CAPREF:
      capRefTick();
      break;

    case ST_TURN:
      if (turnDone()) { motorStop(); capScanIdx = 0; state = ST_CAPNEW; }
      break;

    case ST_CAPNEW:
      capNewTick();
      break;

    case ST_VERIFY:
      verifyAfterTurn();
      break;

    case ST_BACKUP:
      // kept for safety; use DISENGAGE as the primary escape
      enterDisengage();
      break;

    case ST_DISENGAGE:
      disengTick();
      break;

    default:
      break;
  }
}