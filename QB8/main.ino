#include <REG.h>
#include <Servo.h>
#include <TCA9548.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <wit_c_sdk.h>
// Codice 26/04/12 algoritmo Dijkstra con gestione rampe e tutto ben funzionante
// come navigazione

// --- Configurazione BH1745 (Sensore Colore) ---
#define BH1745_ADDR 0x39
#define SYSTEM_CONTROL 0x40
#define MODE_CONTROL1 0x41
#define MODE_CONTROL2 0x42
#define NERO_C_THRESHOLD 250 // Regola questo se hai ancora falsi positivi
#define BLU_B_MIN 1500
#define BLU_B_TO_R_RATIO 10.0
#define BLU_B_TO_G_RATIO 1.4
float fPitchReale = 0;
float pitchIniziale = 0;

// Variabili sparacubetti
Servo servoTorretta;
const int PIN_SERVO_TORRETTA = 11;

// --- IMPOSTAZIONI ---
const int SOGLIA_STOP = 1;
const int SOGLIA_CONFERMA = 3;
const int COOLDOWN_SGANCIO_MS = 1000;

// --- VARIABILI DI STATO ---
unsigned long tempoUltimoSgancio = 0;

char ultimaVittimaCam1 = ' ';
int contatoreCam1 = 0;

char ultimaVittimaCam2 = ' ';
int contatoreCam2 = 0;

// Multiplexer: indirizzo 0x70, su Wire1
TCA9548 mux(0x70, &Wire1);

// Array per 6 sensori (canali 0-5)
VL53L0X sensors[6];

const char *sensorNames[6] = {
    "Destra_giu",   // S0
    "Avanti:giu",   // S1
    "Avanti:su",    // S2
    "Sinsitra:su",  // S3
    "Sinistra:giu", // S4
    "destra:su"     // S5
};

// Variabili WT61P
#define ANGLE_UPDATE 0x04
static char s_cDataUpdate = 0;
float fYawOffset = 0;
float fYawReale = 0;

// PIN MOTORI - TMC2209
#define STEP_SX 1
#define DIR_SX 2
#define MS1_SX 4
#define MS2_SX 5

#define STEP_DX 42
#define DIR_DX 40
#define MS1_DX 36
#define MS2_DX 34

unsigned long lastmicrosdx = 0;
unsigned long lastmicrossx = 0;
float dv = 0;
float vdx = 500;
float vsx = 500;
bool statostepsx = false;
bool statostepdx = false;
float vmstepcounter = 0;
float stepcounter = 0;
#define step_per_cm 50

// Variabili PID
int counter = 0;
float kproporzionale = 100;
float ang = 0;
float corrdx = 0;
float corrsx = 0;
float corr = 0;
float yawcorretto = 0;
float derivate = 0;
float corrPrec;
float corrCalcolo;
bool premutodx = 0;
float timerpremutodx = 0;
bool premutosx = 0;
float timerpremutosx = 0;

// ==========================================
// VARIABILI NAVIGAZIONE E MAPPATURA
// ==========================================
#define MAP_SIZE 51
#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4

// Struttura Cella estesa per supportare i pesi e la sigillatura dei neri
struct Cell {
  bool visited;
  bool wall[5];      // Muri rilevati dai ToF o logici
  bool blackWall[5]; // Muri permanenti causati da nero o rampe chiuse
  bool isBlue;       // Costa 5 passare qui
  bool isBlack;      // Tile invalida (buco nero o rampa chiusa)
  bool victimProcessed;
};

Cell maze[MAP_SIZE][MAP_SIZE];

int x = MAP_SIZE / 2; // Partenza al centro
int y = MAP_SIZE / 2;
int startX = x; // Salviamo la partenza per l'exit bonus
int startY = y;
int direzione = NORTH; // Guarda a NORD inizialmente

bool bluRilevato = false;
bool rampaCorrente = false;

// Prototipi
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data,
                            uint32_t length);
static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data,
                             uint32_t length);
static void Delayms(uint16_t ucMs);
static void AutoScanSensor(void);
void centratiSulMuro();

// ==========================================
// SENSORI COLORE
// ==========================================
void bh1745Write8(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(BH1745_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
}

bool bh1745Read16(uint8_t reg, uint16_t &out) {
  Wire1.beginTransmission(BH1745_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0)
    return false;
  if (Wire1.requestFrom(BH1745_ADDR, (uint8_t)2) != 2)
    return false;
  uint8_t lo = Wire1.read();
  uint8_t hi = Wire1.read();
  out = (uint16_t)lo | ((uint16_t)hi << 8);
  return true;
}

void bh1745Init() {
  bh1745Write8(SYSTEM_CONTROL, 0b10100000);
  delay(50);
  bh1745Write8(SYSTEM_CONTROL, 0b00000000);
  delay(10);
  bh1745Write8(MODE_CONTROL1, 0b00000000);
  bh1745Write8(MODE_CONTROL2, 0b00010010);
}

bool isBlackDetected() {
  mux.selectChannel(6);
  uint16_t c;
  if (bh1745Read16(0x56, c)) {
    if (c < NERO_C_THRESHOLD) {
      Serial.print("!!! NERO RILEVATO !!! Valore C: ");
      Serial.println(c);
    }
    return (c < NERO_C_THRESHOLD);
  }
  return false;
}

bool isBlueDetected() {
  mux.selectChannel(6);
  uint16_t r, g, b;
  if (!bh1745Read16(0x50, r) || !bh1745Read16(0x52, g) ||
      !bh1745Read16(0x54, b))
    return false;
  if (r == 0)
    r = 1;
  if (g == 0)
    g = 1;
  float bToR = (float)b / (float)r;
  float bToG = (float)b / (float)g;
  return (b > BLU_B_MIN && bToR > BLU_B_TO_R_RATIO && bToG > BLU_B_TO_G_RATIO);
}

// ==========================================
// HELPER MAPPA E DIREZIONI
// ==========================================
int opposto(int dir) {
  if (dir == NORTH)
    return SOUTH;
  if (dir == SOUTH)
    return NORTH;
  if (dir == EAST)
    return WEST;
  if (dir == WEST)
    return EAST;
  return 0;
}

int dirDestra(int dir) {
  if (dir == NORTH)
    return EAST;
  if (dir == EAST)
    return SOUTH;
  if (dir == SOUTH)
    return WEST;
  if (dir == WEST)
    return NORTH;
  return 0;
}

int dirSinistra(int dir) {
  if (dir == NORTH)
    return WEST;
  if (dir == EAST)
    return NORTH;
  if (dir == SOUTH)
    return EAST;
  if (dir == WEST)
    return SOUTH;
  return 0;
}

int angoloPerGirare(int dirTarget) {
  if (direzione == dirTarget)
    return 0;
  if (dirDestra(direzione) == dirTarget)
    return -90;
  if (dirSinistra(direzione) == dirTarget)
    return 90;
  return 180;
}

// Aggiornata per non sovrascrivere mai i muri neri permanenti
void setWall(int cx, int cy, int dir, bool hasMuro) {
  if (maze[cx][cy].blackWall[dir])
    return; // Protezione muri permanenti

  maze[cx][cy].wall[dir] = hasMuro;

  int nx = cx;
  int ny = cy;
  if (dir == NORTH)
    ny--;
  if (dir == EAST)
    nx++;
  if (dir == SOUTH)
    ny++;
  if (dir == WEST)
    nx--;

  if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) {
    if (maze[nx][ny].blackWall[opposto(dir)])
      return; // Protezione anche per la cella adiacente
    maze[nx][ny].wall[opposto(dir)] = hasMuro;
  }
}

// Nuova funzione per sigillare completamente buchi neri e rampe chiuse
void setBlackWall(int cx, int cy, int dir) {
  maze[cx][cy].wall[dir] = true;
  maze[cx][cy].blackWall[dir] = true;

  int nx = cx;
  int ny = cy;
  if (dir == NORTH)
    ny--;
  if (dir == EAST)
    nx++;
  if (dir == SOUTH)
    ny++;
  if (dir == WEST)
    nx--;

  if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) {
    // Segnala la tile come esplorata e nera
    maze[nx][ny].visited = true;
    maze[nx][ny].isBlack = true;

    // Sigilla la tile da TUTTI i lati
    for (int i = 1; i <= 4; i++) {
      maze[nx][ny].wall[i] = true;
      maze[nx][ny].blackWall[i] = true;
    }
  }
}

// ==========================================
// ALGORITMO DIJKSTRA PER PATHFINDING PESATO
// ==========================================
bool puoAndare(int dir) {
  if (maze[x][y].wall[dir])
    return false; // C'è un muro fisico o una tile nera sigillata
  int nx = x;
  int ny = y;
  if (dir == NORTH)
    ny--;
  if (dir == EAST)
    nx++;
  if (dir == SOUTH)
    ny++;
  if (dir == WEST)
    nx--;

  if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE)
    return false;
  if (maze[nx][ny].visited)
    return false; // Ignora se già esplorata (o marcata isBlack)
  return true;
}

int dijkstraNextMove(bool returnToStart) {
  int dist[MAP_SIZE][MAP_SIZE];
  int parentDir[MAP_SIZE][MAP_SIZE];

  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      dist[i][j] = 9999;
      parentDir[i][j] = 0;
    }
  }

  dist[x][y] = 0;

  bool q[MAP_SIZE][MAP_SIZE];
  memset(q, false, sizeof(q));
  q[x][y] = true;

  int targetX = -1, targetY = -1;
  int minDistToUnexplored = 9999;

  while (true) {
    int uX = -1, uY = -1;
    int currentMin = 9999;

    for (int i = 0; i < MAP_SIZE; i++) {
      for (int j = 0; j < MAP_SIZE; j++) {
        if (q[i][j] && dist[i][j] < currentMin) {
          currentMin = dist[i][j];
          uX = i;
          uY = j;
        }
      }
    }

    if (uX == -1)
      break;
    q[uX][uY] = false;

    if (!returnToStart) {
      if (!maze[uX][uY].visited) {
        if (dist[uX][uY] < minDistToUnexplored) {
          minDistToUnexplored = dist[uX][uY];
          targetX = uX;
          targetY = uY;
          break;
        }
      }
    } else {
      if (uX == startX && uY == startY) {
        targetX = uX;
        targetY = uY;
        break;
      }
    }

    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};
    int dirs[] = {NORTH, EAST, SOUTH, WEST};

    for (int i = 0; i < 4; i++) {
      int dir = dirs[i];
      if (!maze[uX][uY].wall[dir]) {
        int vX = uX + dx[i];
        int vY = uY + dy[i];

        if (vX >= 0 && vX < MAP_SIZE && vY >= 0 && vY < MAP_SIZE) {
          // I Pesi sono moltiplicati per 10 per lasciare spazio alla penalità
          // di curva
          int weight = maze[vX][vY].isBlue ? 100 : 10;

          // PENALITA' ROTAZIONE: se devo svoltare aggiungo un costo piccolo
          // (+1) Così a parità di distanza, sceglierà il percorso con meno
          // curve possibili.
          int arrivedFrom =
              (uX == x && uY == y) ? direzione : parentDir[uX][uY];
          if (dir != arrivedFrom) {
            weight += 1;
          }

          if (dist[uX][uY] + weight < dist[vX][vY]) {
            dist[vX][vY] = dist[uX][uY] + weight;
            parentDir[vX][vY] = dir;
            q[vX][vY] = true;
          }
        }
      }
    }
  }

  if (targetX == -1)
    return 0;

  int currX = targetX;
  int currY = targetY;
  int moveTarget = 0;

  while (!(currX == x && currY == y)) {
    int cameFromDir = parentDir[currX][currY];
    moveTarget = cameFromDir;

    if (cameFromDir == NORTH)
      currY++;
    else if (cameFromDir == EAST)
      currX--;
    else if (cameFromDir == SOUTH)
      currY--;
    else if (cameFromDir == WEST)
      currX++;
  }

  return moveTarget;
}

int scegliDirezioneAvanzato(bool exitMode) {
  if (!exitMode) {
    // 1. ESPLORAZIONE GREEDY: Andiamo dritto appena possibile!
    // Stessa logica del tuo codice originale per evitare che la griglia sballi
    // le priorità
    if (puoAndare(direzione))
      return direzione;
    if (puoAndare(dirSinistra(direzione)))
      return dirSinistra(direzione);
    if (puoAndare(dirDestra(direzione)))
      return dirDestra(direzione);

    // 2. BACKTRACKING: Se siamo incastrati o circondati da tile già visitate,
    // chiama Dijkstra
    return dijkstraNextMove(false);
  } else {
    // 3. EXIT BONUS: Calcola solo il percorso verso la partenza
    return dijkstraNextMove(true);
  }
}

// ==========================================
// FUNZIONI MOTORI E PID
// ==========================================
void stopMotori() {
  digitalWrite(STEP_SX, LOW);
  digitalWrite(STEP_DX, LOW);
}

void retromarcia(int velocita, long passiDaFare) {
  long passiFattiBack = 0;
  unsigned long lastm_dx = micros();
  unsigned long lastm_sx = micros();
  bool st_dx = false;
  bool st_sx = false;

  digitalWrite(DIR_DX, HIGH);
  digitalWrite(DIR_SX, HIGH);

  while (passiFattiBack < passiDaFare) {
    if (micros() > (lastm_dx + abs(1000000 / velocita))) {
      st_dx = !st_dx;
      digitalWrite(STEP_DX, st_dx);
      lastm_dx = micros();
      if (st_dx == true)
        passiFattiBack++;
    }
    if (micros() > (lastm_sx + abs(1000000 / velocita))) {
      st_sx = !st_sx;
      digitalWrite(STEP_SX, st_sx);
      lastm_sx = micros();
    }
  }
  stopMotori();
}

int gira(int angiro) {
  statostepsx = false;
  statostepdx = false;
  lastmicrossx = 0;
  lastmicrosdx = 0;
  vmstepcounter = 0;
  stepcounter = 0;
  fYawOffset = fYawOffset + angiro;
  ang = fYawReale - fYawOffset;
  counter = millis() + 2000;
  unsigned long lastI2C_Check = micros();

  while (ang < -0.2 || ang > 0.2) {
    if (micros() - lastI2C_Check > 1000) {
      lastI2C_Check = micros();
      WitReadReg(Yaw, 1);
      if (s_cDataUpdate & ANGLE_UPDATE) {
        fYawReale = sReg[Yaw] / 32768.0f * 180.0f;
        s_cDataUpdate &= ~ANGLE_UPDATE;
        ang = fYawReale - fYawOffset;
        while (ang > 180)
          ang -= 360;
        while (ang < -180)
          ang += 360;
        dv = (300 * sqrt(abs(ang)) * (ang / abs(ang)));
        vsx = dv;
        vdx = 0 - dv;
      }
    }
    if (micros() > (lastmicrosdx + abs(1000000 / vdx))) {
      if (vdx < 0)
        digitalWrite(DIR_DX, HIGH);
      else
        digitalWrite(DIR_DX, LOW);
      statostepdx = !statostepdx;
      digitalWrite(STEP_DX, statostepdx);
      lastmicrosdx = micros();
    }
    if (micros() > (lastmicrossx + abs(1000000 / vsx))) {
      if (vsx < 0)
        digitalWrite(DIR_SX, HIGH);
      else
        digitalWrite(DIR_SX, LOW);
      statostepsx = !statostepsx;
      lastmicrossx = micros();
      digitalWrite(STEP_SX, statostepsx);
    }
  }
  return 0;
}

int pidavanti(int vm, int cm) {
  statostepsx = false;
  statostepdx = false;
  lastmicrossx = 0;
  lastmicrosdx = 0;

  corrdx = 0;
  corrsx = 0;
  corr = 0;
  counter = millis();
  yawcorretto = fYawOffset;
  uint16_t d0raw = 999;
  uint16_t d4raw = 999;

  long targetStepsOrizzontali = cm * step_per_cm;
  float stepOrizzontaliFatti = 0.0;

  float diffPitchRaw = fPitchReale - pitchIniziale;
  float diffPitch = abs(diffPitchRaw);

  long stepFisiciPiano = 0;
  long stepFisiciRampa = 0;

  bool rampaConfermata = false;
  bool inInclinazione = false;
  bool salitaConfermata = false;

  unsigned long timerInclinazione = 0;
  unsigned long timerUscitaRampa = millis();

  float sommaGradiRampa = 0.0;
  long campioniGradiRampa = 0;
  float angoloMedio = 0.0;

  while (true) {
    if (stepOrizzontaliFatti >= targetStepsOrizzontali) {
      if (rampaConfermata) {
        targetStepsOrizzontali += (30 * step_per_cm);
      } else {
        break;
      }
    }

    // Sparacubetti ->

    if (millis() - tempoUltimoSgancio < COOLDOWN_SGANCIO_MS) {
      while (Serial.available() > 0) {
        Serial.read();
      }
    } else if (Serial.available() >= 2) {
      char cam = Serial.read();
      char vittima = Serial.read();

      bool muroSX = (d4raw < 160);
      bool muroDX = (d0raw < 160);
      bool latoValido = (cam == '1' && muroSX) || (cam == '2' && muroDX);

      if (latoValido && maze[x][y].victimProcessed == false) {

        // ==========================================
        // GESTIONE CAM 1 (SINISTRA)
        // ==========================================
        if (cam == '1') {
          if (vittima == 'H' || vittima == 'S' || vittima == 'U') {
            contatoreCam1++;
          }

          if (contatoreCam1 >= SOGLIA_STOP) {
            stopMotori();

            int countH = 0, countS = 0, countU = 0;
            unsigned long timeout = millis() + 2000;
            int lettureRaccolte = 0;

            while (Serial.available() > 0)
              Serial.read();

            while (millis() < timeout && lettureRaccolte < 15) {
              if (Serial.available() >= 2) {
                char checkCam = Serial.read();
                char checkVittima = Serial.read();

                if (checkCam == '1') {
                  if (checkVittima == 'H')
                    countH++;
                  else if (checkVittima == 'S')
                    countS++;
                  else if (checkVittima == 'U')
                    countU++;
                  lettureRaccolte++;
                }
              }
            }

            char vincitore = ' ';
            int maxCount = 0;
            if (countH > maxCount) {
              maxCount = countH;
              vincitore = 'H';
            }
            if (countS > maxCount) {
              maxCount = countS;
              vincitore = 'S';
            }
            if (countU > maxCount) {
              maxCount = countU;
              vincitore = 'U';
            }

            if (maxCount >= SOGLIA_CONFERMA) {
              sganciaKitSinistra(vincitore);
              maze[x][y].victimProcessed = true;
            } else {
              Serial.print("Falso allarme Cam1 - max:");
              Serial.print(maxCount);
              Serial.print(" tipo:");
              Serial.println(vincitore);
            }

            resettaTutto();
            lastmicrosdx = micros();
            lastmicrossx = micros();
            counter = millis();
          }
        }

        // ==========================================
        // GESTIONE CAM 2 (DESTRA)
        // ==========================================
        else if (cam == '2') {
          if (vittima == 'H' || vittima == 'S' || vittima == 'U') {
            contatoreCam2++;
          }

          if (contatoreCam2 >= SOGLIA_STOP) {
            stopMotori();

            int countH = 0, countS = 0, countU = 0;
            unsigned long timeout = millis() + 2000;
            int lettureRaccolte = 0;

            while (Serial.available() > 0)
              Serial.read();

            while (millis() < timeout && lettureRaccolte < 15) {
              if (Serial.available() >= 2) {
                char checkCam = Serial.read();
                char checkVittima = Serial.read();

                if (checkCam == '2') {
                  if (checkVittima == 'H')
                    countH++;
                  else if (checkVittima == 'S')
                    countS++;
                  else if (checkVittima == 'U')
                    countU++;
                  lettureRaccolte++;
                }
              }
            }

            char vincitore = ' ';
            int maxCount = 0;
            if (countH > maxCount) {
              maxCount = countH;
              vincitore = 'H';
            }
            if (countS > maxCount) {
              maxCount = countS;
              vincitore = 'S';
            }
            if (countU > maxCount) {
              maxCount = countU;
              vincitore = 'U';
            }

            if (maxCount >= SOGLIA_CONFERMA) {
              sganciaKitDestra(vincitore);
              maze[x][y].victimProcessed = true;
            } else {
              Serial.print("Falso allarme Cam2 - max:");
              Serial.print(maxCount);
              Serial.print(" tipo:");
              Serial.println(vincitore);
            }

            resettaTutto();
            lastmicrosdx = micros();
            lastmicrossx = micros();
            counter = millis();
          }
        }
      } else {
        contatoreCam1 = 0;
        contatoreCam2 = 0;
      }
    }

    //<- Sparacubetti

    WitReadReg(Pitch, 2);
    if (s_cDataUpdate & ANGLE_UPDATE) {
      fPitchReale = sReg[Pitch] / 32768.0f * 180.0f;
      fYawReale = sReg[Yaw] / 32768.0f * 180.0f;
      s_cDataUpdate &= ~ANGLE_UPDATE;

      diffPitchRaw = fPitchReale - pitchIniziale;
      diffPitch = abs(diffPitchRaw);

      if (diffPitch > 10.0) {
        timerUscitaRampa = millis();
        if (!inInclinazione) {
          inInclinazione = true;
          timerInclinazione = millis();
        } else if (!rampaConfermata && (millis() - timerInclinazione > 350)) {
          rampaConfermata = true;
          sommaGradiRampa = 0.0;
          campioniGradiRampa = 0;
          salitaConfermata = (diffPitchRaw < 0);
        }
      } else {
        inInclinazione = false;
        if (rampaConfermata && (millis() - timerUscitaRampa > 100)) {
          rampaConfermata = false;
        }
      }

      if (rampaConfermata) {
        sommaGradiRampa += diffPitch;
        campioniGradiRampa++;
        angoloMedio = sommaGradiRampa / campioniGradiRampa;
      }

      ang = fYawReale - yawcorretto;
      while (ang > 180)
        ang -= 360;
      while (ang < -180)
        ang += 360;

      dv = (kproporzionale * ang);
      vsx = vm + dv;
      vdx = vm - dv;
    }

    if (millis() > (counter + 150)) {
      counter = millis();
      if (!rampaConfermata) {
        if (isBlackDetected()) {
          stopMotori();
          delay(100);
          retromarcia(vm / 2, (stepFisiciPiano));
          return -1; // Nero rilevato
        }
        if (isBlueDetected()) {
          bluRilevato = true;
        } else {
          bluRilevato = false;
        }
      }

      mux.selectChannel(1);
      uint16_t distAvanti = sensors[1].readRangeContinuousMillimeters();
      if (rampaConfermata) {
        if (distAvanti < 75) {
          stopMotori();
          delay(500);
          int tileSicureFinora =
              floor((stepOrizzontaliFatti / step_per_cm) / 30.0);
          long passiIndietro = stepFisiciPiano;
          if (salitaConfermata) {
            passiIndietro += (stepFisiciRampa / 1.25);
          } else {
            passiIndietro += (stepFisiciRampa * 1.25);
          }
          retromarcia(vm, passiIndietro);
          return -(10 + tileSicureFinora); // Vicolo cieco su rampa
        }
      }

      mux.selectChannel(0);
      d0raw = sensors[0].readRangeContinuousMillimeters() - 25;
      if (!sensors[0].timeoutOccurred()) {
        mux.selectChannel(4);
        d4raw = sensors[4].readRangeContinuousMillimeters() - 16;
        if (!sensors[4].timeoutOccurred()) {
          if (d0raw < 160) {
            corrdx = d0raw - 85;
          } else {
            if (d4raw < 160)
              corrdx = 85 - d4raw;
          }
          if (d4raw < 160) {
            corrsx = d4raw - 90;
          } else {
            if (d0raw < 160) {
              corrsx = 90 - d0raw;
            }
          }

          corrCalcolo = (corrdx - corrsx);
          derivate = corrCalcolo - corrPrec;

          if (d4raw > 160 && d0raw > 160)
            corr = 0;
          else
            corr = 0 - corrCalcolo * 0.1 - derivate * 0.2;

          yawcorretto = fYawOffset + corr * 0.4;
          corrPrec = corrCalcolo;

          while (yawcorretto > 180)
            yawcorretto -= 360;
          while (yawcorretto < -180)
            yawcorretto += 360;
        }
      }
    }

    // controllo microswitch
    if (!rampaConfermata) {
      if (!bluRilevato) {
        if (digitalRead(12) == 1 && digitalRead(13) == 0) {
          if (premutodx == 0) {
            premutodx = 1;
            timerpremutodx = millis();
          } else {
            if (timerpremutodx + 150 < millis()) {
              digitalWrite(DIR_DX, HIGH);
              digitalWrite(DIR_SX, HIGH);
              for (int i5 = 0; i5 < 250; i5++) {
                digitalWrite(STEP_DX, HIGH);
                digitalWrite(STEP_SX, HIGH);
                delayMicroseconds(700);
                digitalWrite(STEP_DX, LOW);
                digitalWrite(STEP_SX, LOW);
                delayMicroseconds(700);
              }
              mux.selectChannel(1);
              uint16_t distAvantiBasso =
                  sensors[1].readRangeContinuousMillimeters();
              if (distAvantiBasso < 300) {
                bool muroAvanti = true;
              } else {
                gira(20);
                digitalWrite(DIR_DX, LOW);
                digitalWrite(DIR_SX, LOW);
                for (int i5 = 0; i5 < 250; i5++) {
                  digitalWrite(STEP_DX, HIGH);
                  digitalWrite(STEP_SX, HIGH);
                  delayMicroseconds(700);
                  digitalWrite(STEP_DX, LOW);
                  digitalWrite(STEP_SX, LOW);
                  delayMicroseconds(700);
                }
                gira(-20);
              }
            }
          }
        } else {
          premutodx = 0;
          timerpremutodx = millis();
        }

        if (digitalRead(12) == 0 && digitalRead(13) == 1) {
          if (premutosx == 0) {
            premutosx = 1;
            timerpremutosx = millis();
          } else {
            if (timerpremutosx + 150 < millis()) {
              digitalWrite(DIR_DX, HIGH);
              digitalWrite(DIR_SX, HIGH);
              for (int i5 = 0; i5 < 250; i5++) {
                digitalWrite(STEP_DX, HIGH);
                digitalWrite(STEP_SX, HIGH);
                delayMicroseconds(700);
                digitalWrite(STEP_DX, LOW);
                digitalWrite(STEP_SX, LOW);
                delayMicroseconds(700);
              }
              mux.selectChannel(1);
              uint16_t distAvantiBasso =
                  sensors[1].readRangeContinuousMillimeters();
              if (distAvantiBasso < 300) {
                bool muroAvanti = true;
              } else {
                gira(-20);
                digitalWrite(DIR_DX, LOW);
                digitalWrite(DIR_SX, LOW);
                for (int i5 = 0; i5 < 250; i5++) {
                  digitalWrite(STEP_DX, HIGH);
                  digitalWrite(STEP_SX, HIGH);
                  delayMicroseconds(700);
                  digitalWrite(STEP_DX, LOW);
                  digitalWrite(STEP_SX, LOW);
                  delayMicroseconds(700);
                }
                gira(20);
              }
            }
          }
        } else {
          premutosx = 0;
          timerpremutosx = millis();
        }
      }
    }

    if (vdx == 0)
      vdx = 1;
    if (micros() > (lastmicrosdx + abs(1000000 / vdx))) {
      if (vdx < 0)
        digitalWrite(DIR_DX, HIGH);
      else
        digitalWrite(DIR_DX, LOW);
      statostepdx = !statostepdx;
      digitalWrite(STEP_DX, statostepdx);
      lastmicrosdx = micros();

      if (statostepdx == true) {
        if (rampaConfermata) {
          stepFisiciRampa++;
          float calcoloAngolo = (angoloMedio > 25.0) ? 25.0 : angoloMedio;
          float avanzamento = cos(calcoloAngolo * (PI / 180.0));
          if (salitaConfermata)
            avanzamento = avanzamento / 1.25;
          stepOrizzontaliFatti += avanzamento;
        } else {
          stepFisiciPiano++;
          stepOrizzontaliFatti += 1.0;
        }
      }
    }

    if (vsx == 0)
      vsx = 1;
    if (micros() > (lastmicrossx + abs(1000000 / vsx))) {
      if (vsx < 0)
        digitalWrite(DIR_SX, HIGH);
      else
        digitalWrite(DIR_SX, LOW);
      statostepsx = !statostepsx;
      lastmicrossx = micros();
      digitalWrite(STEP_SX, statostepsx);
    }
  }

  stopMotori();
  fYawOffset = yawcorretto;

  if (isBlueDetected()) {
    bluRilevato = true;
  }

  rampaCorrente = rampaConfermata;
  return round((stepOrizzontaliFatti / step_per_cm) / 30.0);
}

void centratiSulMuro() {
  int target_mm = 37;
  mux.selectChannel(1);
  uint16_t checkDist = sensors[1].readRangeContinuousMillimeters() - 10;

  if (checkDist < 250) {
    mux.selectChannel(1);
    uint16_t distFront = sensors[1].readRangeContinuousMillimeters();

    long passi = step_per_cm * abs(distFront - target_mm) * 0.1;
    if (distFront > target_mm) {
      digitalWrite(DIR_DX, LOW);
      digitalWrite(DIR_SX, LOW);
    } else {
      digitalWrite(DIR_DX, HIGH);
      digitalWrite(DIR_SX, HIGH);
    }

    for (long i = 0; i < passi; i++) {
      digitalWrite(STEP_DX, HIGH);
      digitalWrite(STEP_SX, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_DX, LOW);
      digitalWrite(STEP_SX, LOW);
      delayMicroseconds(500);
    }
    stopMotori();
    delay(200);
  }
}

int pidgira(int angiro) {
  if (angiro == -90)
    direzione = direzione + 1;
  if (angiro == 90)
    direzione = direzione - 1;
  if (angiro == 180 || angiro == -180)
    direzione = direzione + 2;

  if (direzione > 4)
    direzione -= 4;
  if (direzione < 1)
    direzione += 4;

  statostepsx = false;
  statostepdx = false;
  lastmicrossx = 0;
  lastmicrosdx = 0;
  vmstepcounter = 0;
  stepcounter = 0;
  fYawOffset = fYawOffset + angiro;
  ang = fYawReale - fYawOffset;
  counter = millis() + 2000;

  unsigned long startGiro = millis();
  unsigned long lastI2C_Check = micros();

  while (ang < -0.2 || ang > 0.2) {
    if (millis() - startGiro > 10000) {
      stopMotori();
      return 1;
    }
    if (micros() - lastI2C_Check > 1000) {
      lastI2C_Check = micros();
      WitReadReg(Yaw, 1);
      if (s_cDataUpdate & ANGLE_UPDATE) {
        fYawReale = sReg[Yaw] / 32768.0f * 180.0f;
        s_cDataUpdate &= ~ANGLE_UPDATE;
        ang = fYawReale - fYawOffset;
        while (ang > 180)
          ang -= 360;
        while (ang < -180)
          ang += 360;
        dv = (300 * sqrt(abs(ang)) * (ang / abs(ang)));
        vsx = dv;
        vdx = 0 - dv;
      }
    }
    if (micros() > (lastmicrosdx + abs(1000000 / vdx))) {
      if (vdx < 0)
        digitalWrite(DIR_DX, HIGH);
      else
        digitalWrite(DIR_DX, LOW);
      statostepdx = !statostepdx;
      digitalWrite(STEP_DX, statostepdx);
      lastmicrosdx = micros();
    }
    if (micros() > (lastmicrossx + abs(1000000 / vsx))) {
      if (vsx < 0)
        digitalWrite(DIR_SX, HIGH);
      else
        digitalWrite(DIR_SX, LOW);
      statostepsx = !statostepsx;
      lastmicrossx = micros();
      digitalWrite(STEP_SX, statostepsx);
    }
  }

  centratiSulMuro();
  return 0;
}

void initSensor(VL53L0X *sensor, uint8_t channel) {
  mux.selectChannel(channel);
  sensor->setTimeout(0);
  if (!sensor->init()) {
    Serial.print("ERRORE S");
    Serial.println(channel);
  } else {
    sensor->startContinuous();
    Serial.print("S");
    Serial.print(channel);
    Serial.println(" OK!");
  }
}

// Funzioni sparacubetti
void torretta_sinistra() { servoTorretta.write(0); }

void torretta_destra() { servoTorretta.write(180); }

void torretta_centroDX() { servoTorretta.write(80); }
void torretta_centroSX() { servoTorretta.write(100); }

void lampeggia(int pin) {
  digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i < 5; i++) {
    digitalWrite(pin, LOW);
    delay(500);
    digitalWrite(pin, HIGH);
    delay(500);
  }
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void sganciaKitSinistra(char tipoVittima) {
  Serial.print("--- SGANCIO A SINISTRA! Vittima: ");
  Serial.println(tipoVittima);

  if (tipoVittima == 'U') {
    // Nessuno sparo
    lampeggia(86); // ← 5 secondi

  } else if (tipoVittima == 'S') {
    // 1 cubetto
    lampeggia(86); // ← 5 secondi
    torretta_sinistra();
    delay(600);
    torretta_centroSX();
    delay(600);

  } else if (tipoVittima == 'H') {
    // 2 cubetti
    lampeggia(86); // ← 5 secondi
    torretta_sinistra();
    delay(600);
    torretta_centroSX();
    delay(600);
    torretta_sinistra();
    delay(600);
    torretta_centroSX();
    delay(600);
  }
}

void sganciaKitDestra(char tipoVittima) {
  Serial.print("--- SGANCIO A DESTRA! Vittima: ");
  Serial.println(tipoVittima);

  if (tipoVittima == 'U') {
    // Nessuno sparo
    lampeggia(86); // ← 5 secondi

  } else if (tipoVittima == 'S') {
    // 1 cubetto
    lampeggia(86); // ← 5 secondi
    torretta_destra();
    delay(600);
    torretta_centroDX();
    delay(600);

  } else if (tipoVittima == 'H') {
    // 2 cubetti
    lampeggia(86); // ← 5 secondi
    torretta_destra();
    delay(600);
    torretta_centroDX();
    delay(600);
    torretta_destra();
    delay(600);
    torretta_centroDX();
    delay(600);
  }
}

void resettaTutto() {
  tempoUltimoSgancio = millis();
  contatoreCam1 = 0;
  ultimaVittimaCam1 = ' ';
  contatoreCam2 = 0;
  ultimaVittimaCam2 = ' ';
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  servoTorretta.attach(PIN_SERVO_TORRETTA);
  servoTorretta.write(90);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  Serial.begin(115200);

  mux.begin();
  Serial.println("TCA9548 pronto su Wire1!");

  for (int i = 0; i < 6; i++)
    initSensor(&sensors[i], i);

  mux.selectChannel(6);
  bh1745Init();

  WitInit(WIT_PROTOCOL_I2C, 0x50);
  WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
  WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);
  AutoScanSensor();

  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(36, LOW);
  digitalWrite(34, LOW);
  digitalWrite(2, LOW);
  digitalWrite(40, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("Calibrazione in corso");
  delay(1000);
  Serial.println("Calibrazione giroscopio in corso...");
  delay(1000);

  float sommaYaw = 0;
  float sommaPitch = 0;
  for (int i = 0; i < 10; i++) {
    WitReadReg(Pitch, 2);
    delay(20);
    if (s_cDataUpdate & ANGLE_UPDATE) {
      sommaPitch += (sReg[Pitch] / 32768.0f * 180.0f);
      sommaYaw += (sReg[Yaw] / 32768.0f * 180.0f);
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
  }
  pitchIniziale = sommaPitch / 10.0;
  fYawOffset = sommaYaw / 10.0;

  // Inizializza Start Mappa
  maze[x][y].visited = true;
  delay(2000);
}

// ==========================================
// LOOP PRINCIPALE (MAPPATURA + DIJKSTRA)
// ==========================================
void loop() {
  mux.selectChannel(1);
  uint16_t distAvantiBasso = sensors[1].readRangeContinuousMillimeters();
  mux.selectChannel(2);
  uint16_t distAvantiAlto = sensors[2].readRangeContinuousMillimeters();
  mux.selectChannel(0);
  uint16_t distDestraBasso = sensors[0].readRangeContinuousMillimeters();
  mux.selectChannel(5);
  uint16_t distDestraAlto = sensors[5].readRangeContinuousMillimeters();
  mux.selectChannel(4);
  uint16_t distSinistraBasso = sensors[4].readRangeContinuousMillimeters();
  mux.selectChannel(3);
  uint16_t distSinistraAlto = sensors[3].readRangeContinuousMillimeters();

  bool muroAvanti = false;
  if (distAvantiAlto < 250) {
    if (abs(distAvantiAlto - distAvantiBasso) < 25)
      muroAvanti = true;
  }

  bool muroDestra = false;
  if (distDestraAlto < 250) {
    if (abs(distDestraAlto - distDestraBasso) < 25)
      muroDestra = true;
  }

  bool muroSinistra = false;
  if (distSinistraAlto < 250) {
    if (abs(distSinistraAlto - distSinistraBasso) < 25)
      muroSinistra = true;
  }

  // Aggiorna Mappa Muri (salta se sei su una rampa fisica attualmente)
  if (!rampaCorrente) {
    setWall(x, y, direzione, muroAvanti);
    setWall(x, y, dirDestra(direzione), muroDestra);
    setWall(x, y, dirSinistra(direzione), muroSinistra);
  }

  // DECISION MAKING CON DIJKSTRA
  // DECISION MAKING IBRIDO (Greedy + Dijkstra)
  int prossima = scegliDirezioneAvanzato(false);

  if (prossima == 0) {
    Serial.println("LABIRINTO COMPLETATO! Ritorno alla partenza...");
    prossima = scegliDirezioneAvanzato(true);

    if (prossima == 0 && x == startX && y == startY) {
      stopMotori();
      Serial.println("EXIT BONUS RAGGIUNTO!");
      while (1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
      }
    }
  }

  if (prossima != 0) {

    if (prossima != direzione) {
      centratiSulMuro();
      int angolo = angoloPerGirare(prossima);

      // Salva lo stato PRIMA che pidgira lo modifichi
      int direzioneBackup = direzione;
      float yawOffsetBackup = fYawOffset;

      int esitoGiro = 0;
      if (angolo == 180) {
        esitoGiro = pidgira(90);
        if (esitoGiro == 0) {
          delay(300);
          esitoGiro = pidgira(90);
          if (esitoGiro == 1) {
            pidgira(-90);
            pidgira(-90);
          }
        } else {
          pidgira(-90);
        }
      } else {
        esitoGiro = pidgira(angolo);
        if (esitoGiro == 1)
          pidgira(-angolo);
      }

      if (esitoGiro == 1) {
        Serial.println("ERRORE: INCASTRATO in Rotazione!");

        // Ripristina direzione e yaw a prima del tentativo
        direzione = direzioneBackup;
        fYawOffset = yawOffsetBackup;

        // Segna il muro PRIMA di muoverti (posizione ancora corretta)
        setBlackWall(x, y, prossima);

        // Retromarcia mezza tile — resti nella stessa tile logica, zero rischio
        // mappa
        retromarcia(1500, step_per_cm * 15);

        return;
      }
      delay(300);
    }

    // Avanza
    bluRilevato = false;
    int risultato = pidavanti(1700, 32);

    // Gestione Nero
    if (risultato == -1) {
      setBlackWall(x, y, direzione);
      return;
    }

    // Gestione Rampa verso Muro
    else if (risultato <= -10) {
      setBlackWall(x, y, direzione);
      return;
    }

    // Successo (avanzamento effettivo)
    if (risultato > 0) {
      for (int i = 0; i < risultato; i++) {
        if (direzione == NORTH)
          y--;
        if (direzione == EAST)
          x++;
        if (direzione == SOUTH)
          y++;
        if (direzione == WEST)
          x--;

        maze[x][y].visited = true;
        if (bluRilevato)
          maze[x][y].isBlue = true; // Salva la tile come blu

        // --- FIX PORTE FANTASMA SULLE RAMPE ---
        // Se attraversiamo una tile "volando" (perché pidavanti fa più tile
        // assieme), non faremo mai il setWall dal void loop per questa tile.
        // Siccome le rampe hanno SEMPRE le sponde laterali, le sigilliamo
        // artificialmente!
        if (i <
            risultato - 1) { // i < risultato - 1 identifica le tile intermedie
          setWall(x, y, dirDestra(direzione), true);
          setWall(x, y, dirSinistra(direzione), true);
        }
      }
    }

    // Routine segnalazione Blu
    if (bluRilevato) {
      stopMotori();
      digitalWrite(LED_BUILTIN, HIGH);
      for (int i = 0; i < 5; i++) {
        digitalWrite(88, LOW);
        delay(500);
        digitalWrite(88, HIGH);
        delay(500);
      }
      digitalWrite(LED_BUILTIN, LOW);
      bluRilevato = false;
    }
  }
}

// Funzioni WitMotion I2C
static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data,
                            uint32_t length) {
  int val;
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission(false);
  val = Wire.requestFrom(dev, (int)length);
  if (val == 0)
    return 0;
  uint32_t timeout = millis() + 5;
  while (Wire.available() < length) {
    if (millis() > timeout)
      return 0;
  }
  for (int x = 0; x < length; x++)
    data[x] = Wire.read();
  return 1;
}

static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data,
                             uint32_t length) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  for (uint32_t i = 0; i < length; i++)
    Wire.write(data[i]);
  Wire.endTransmission();
  return 1;
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {
  if (uiReg == Pitch || uiReg == Yaw)
    s_cDataUpdate |= ANGLE_UPDATE;
}

static void Delayms(uint16_t ucMs) { delay(ucMs); }

static void AutoScanSensor(void) {
  int i, iRetry;
  for (i = 0; i < 0x7F; i++) {
    WitInit(WIT_PROTOCOL_I2C, i);
    iRetry = 2;
    do {
      s_cDataUpdate = 0;
      WitReadReg(Yaw, 1);
      delay(5);
      if (s_cDataUpdate != 0)
        return;
      iRetry--;
    } while (iRetry);
  }
}