//***********************************
// PARTS Common Robot Platform
// Test Program
//***********************************

#define PRINT_ENCODER_SPEED 1

const int PIN_XD_EN   =  2;
const int PIN_RD_PWM1 =  3;
const int PIN_LD_PWM1 =  4;
const int PIN_RD_PWM2 =  5;
const int PIN_LD_PWM2 =  6;
const int PIN_LENCB   =  7;
const int PIN_LENCA   =  8;
const int PIN_US_TRIG =  9;
const int PIN_VBAT    = 14;
const int PIN_US_ECHO = 15;
const int PIN_LD_OCM  = 20;
const int PIN_RD_OCM  = 21;
const int PIN_RENCB   = 22;
const int PIN_RENCA   = 23;

#define RENC 0
#define LENC 1

long encCount[2] = {0, 0};
float lastEncSpeed[2] = {0, 0};
const char *wheelName[2] = {"Right", "Left"};

void setup()
{
	// Enable encoder pullups
	pinMode(PIN_LENCA, INPUT_PULLUP);
	pinMode(PIN_LENCB, INPUT_PULLUP);
	pinMode(PIN_RENCA, INPUT_PULLUP);
	pinMode(PIN_RENCB, INPUT_PULLUP);

	// Enable motor drive outputs
	pinMode(PIN_XD_EN, OUTPUT);
	pinMode(PIN_RD_PWM1, OUTPUT);
	pinMode(PIN_LD_PWM1, OUTPUT);
	pinMode(PIN_RD_PWM2, OUTPUT);
	pinMode(PIN_LD_PWM2, OUTPUT);

	// Make sure motors are off to start
	digitalWrite(PIN_XD_EN, HIGH);
	analogWrite(PIN_RD_PWM1, 0);
	analogWrite(PIN_RD_PWM2, 0);
	analogWrite(PIN_LD_PWM1, 0);
	analogWrite(PIN_LD_PWM2, 0);
}

void loop()
{
	static int8_t lastEnc[2] = {0, 0};
	static unsigned long lastEncTime[2] = {0, 0};
	static int dir[2] = {1, 1};
	static bool first = true;
	unsigned long nowMicros = micros();
	const unsigned long nowMillis = millis();
	static long dt[2] = {0, 0};
	int8_t enc[2] = {0, 0};
	float speed[2];
	int i;

	if (digitalRead(PIN_LENCB) == HIGH)
		enc[LENC] |= 1;
	if (digitalRead(PIN_LENCA) == HIGH) {
		enc[LENC] ^= 3;
	}
	if (digitalRead(PIN_RENCA) == HIGH)
		enc[RENC] |= 1;
	if (digitalRead(PIN_RENCB) == HIGH) {
		enc[RENC] ^= 3;
	}

	if (first) {
		first = false;
		lastEnc[0] = enc[0];
		lastEnc[1] = enc[1];
	}

	for (i = 0; i < 2; i++) {
		if (enc[i] != lastEnc[i]) {
			// 1440 counts per revolution
			// 70mm diameter
			// 1 revolution = Pi * 70mm = 220 mm / rev
			// 1 count = Pi * 70mm / 1440 = 153 um / count

			dt[i] = nowMicros - lastEncTime[i];
			lastEncTime[i] = nowMicros;

			dir[i] = enc[i] - lastEnc[i] - 2;
			lastEnc[i] = enc[i];

			if (dir[i] < -2)
				dir[i] += 4;

			encCount[i] += dir[i];

			speed[i] = 153. / dt[i]; // micrometers / microsecond => m/s
			lastEncSpeed[i] = speed[i] * dir[i];
		}
	}

	// print distance and speed
	for (i = 0; i < 2; i++) {
		static unsigned long nextPrintRead[2] = {0, 0};

		if (nowMillis > nextPrintRead[i]) {
			Serial.print(wheelName[i]);
			Serial.print(" ");
			Serial.print(dt[i] / 1e3);
			Serial.print(" msec, dir ");
			Serial.print(dir[i]);
			Serial.print(" total ");
			Serial.print(encCount[i] * 153e-6);
			Serial.print(" meters, speed ");
			Serial.print(lastEncSpeed[i]);
			Serial.println(" m/s");
			nextPrintRead[i] = nowMillis + 1000;
		}
	}
}
