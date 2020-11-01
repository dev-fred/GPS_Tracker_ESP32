/*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SSD1306.h>
#include <OLEDDisplayUi.h>
#include "droneID_FR.h"

extern "C" {
	#include "esp_wifi.h"
	esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
}

//ESP32 PINOUT

#define SSD1306_ADDRESS 0x3c
#define I2C_SDA             21
#define I2C_SCL             22

#define GPS_TX             16
#define GPS_RX             17

#define ENABLE_BUZZER
#define BUZZER_PIN         23

SSD1306 oled(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);
OLEDDisplayUi ui(&oled);

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

droneIDFR drone_idfr;

uint8_t VMAX = 0;


/********************************************************************************************************************
	* MODIFIEZ LES VALEURS ICI
********************************************************************************************************************/
// Set these to your desired credentials.
/**
	* Le nom du point d'acces wifi CHANGEZ LE par ce que vous voulez !!!
*/
//const char ssid[] = "ILLEGAL_DRONE_AP";
const char ssid[] = "AP123456789101112131415";

// Mot de pass du wifi
const char *password = "123456789";
/**
	* CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
*/
const char drone_id[] = "ILLEGAL_DRONE_APPELEZ_POLICE17";


/********************************************************************************************************************/
// NE PAS TOUCHEZ A PARTIR D'ICI !!!
// Le wifi est sur le channel 6 conformement à la spécification
static constexpr uint8_t wifi_channel = 6;
// Ensure the drone_id is max 30 letters
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");
// Ensure the drone_id is max 30 letters
static_assert((sizeof(drone_id)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination
// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;  // default beaconPacket size + max ssid size + max drone id frame size
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
	0x80, 0x00,							            // 0-1: Frame Control
	0x00, 0x00,							            // 2-3: Duration
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,				// 4-9: Destination address (broadcast)
	0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 10-15: Source address FAKE  // TODO should bet set manually
	0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 16-21: Source address FAKE
	0x00, 0x00,							            // 22-23: Sequence / fragment number (done by the SDK)
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,	// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
	0xB8, 0x0B,							            // 32-33: Beacon interval: set to 3s == 3000TU== BB8, bytes in reverse order  // TODO: manually set it
	0x21, 0x04,							            // 34-35: Capability info
	0x03, 0x01, 0x06,						        // 36-38: DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
	0x00, 0x20,                     				// 39-40: SSID parameter set, 0x20:maxlength:content
	// 41-XX: SSID (max 32)
};

double home_alt = 0.0;
int nb_sat = 0;
uint8_t program = 0;
bool ssd1306_found = false;
uint64_t dispMap = 0;
String dispInfo;
char buff[5][256];
uint64_t gpsSec = 0;
uint64_t beaconSec = 0;

#define ARRARY_SIZE(a)   (sizeof(a) / sizeof(a[0]))

void buzz(int targetPin, long frequency, long length) {
	#ifdef ENABLE_BUZZER
		long delayValue = 1000000/frequency/2;
		long numCycles = frequency * length/ 1000;
		for (long i=0; i < numCycles; i++)
		{
			digitalWrite(targetPin,HIGH);
			delayMicroseconds(delayValue);
			digitalWrite(targetPin,LOW);
			delayMicroseconds(delayValue);
		}
	#endif
}

// utilisation
//buzz(4, 2500, 1000); // buzz sur pin 4 à 2500Hz

//Display on SSD1306

void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
	
	display->setFont(ArialMT_Plain_10);
	display->setTextAlignment(TEXT_ALIGN_CENTER);
	
	if (!gps.location.isValid()) {
		display->drawString(64 + x, 11 + y, buff[0]);
		display->drawString(64 + x, 22 + y, buff[1]);
		} else {
		display->drawString(64 + x, 11 + y, buff[0]);
		display->drawString(64 + x, 22 + y, buff[1]);
		display->drawString(64 + x, 33 + y, buff[2]);
		display->drawString(64 + x, 44 + y, buff[3]);
	}
}

FrameCallback frames[] = {drawFrame};


void ssd1306_init()
{
	
	if (!ssd1306_found) {
		Serial.println("SSD1306 not found");
		return;
	}
	if (oled.init()) {
		oled.flipScreenVertically();
		oled.setFont(ArialMT_Plain_16);
		oled.setTextAlignment(TEXT_ALIGN_CENTER);
		} else {
		Serial.println("SSD1306 Begin FAIL");
	}
	Serial.println("SSD1306 Begin PASS");
	
	ui.setTargetFPS(30);
	ui.disableAutoTransition();
	ui.setIndicatorPosition(BOTTOM);
	ui.setIndicatorDirection(LEFT_RIGHT);
	ui.setFrameAnimation(SLIDE_LEFT);
	ui.setFrames(frames, ARRARY_SIZE(frames));
	
}


void scanI2Cdevice(void)
{
	byte err, addr;
	int nDevices = 0;
	for (addr = 1; addr < 127; addr++) {
		Wire.beginTransmission(addr);
		err = Wire.endTransmission();
		if (err == 0) {
			Serial.print("I2C device found at address 0x");
			if (addr < 16)
			Serial.print("0");
			Serial.print(addr, HEX);
			Serial.println(" !");
			nDevices++;
			
			if (addr == SSD1306_ADDRESS) {
				ssd1306_found = true;
				Serial.println("ssd1306 display found");
			}
			
			} else if (err == 4) {
			Serial.print("Unknow error at address 0x");
			if (addr < 16)
			Serial.print("0");
			Serial.println(addr, HEX);
		}
	}
	if (nDevices == 0)
	Serial.println("No I2C devices found\n");
	else
	Serial.println("done\n");
}


/**
	* Phase de configuration.
*/
void setup()
{
	Serial.begin(115200);
	delay(1000);
	Wire.begin(I2C_SDA, I2C_SCL);
	scanI2Cdevice();
	ssd1306_init();
	
	#ifdef ENABLE_BUZZER
		pinMode(BUZZER_PIN, OUTPUT); // set a pin for buzzer output
	#endif
	
	
	
	buzz(BUZZER_PIN, 2500, 100);
	
	SerialGPS.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
	
	/********************************************************************************************************************
		* ICI ON INITIALISE LE WIFI
	*/
	/**
		* Pour mon exemple, je crée un point d'accés. Il fait rien par defaut.
	*/
	Serial.println("Starting AP");
	WiFi.softAP(ssid, nullptr, wifi_channel);
	IPAddress myIP = WiFi.softAPIP();
	Serial.print("AP IP address: ");
	Serial.println(myIP);
	Serial.print("AP mac address: ");
	Serial.println(WiFi.macAddress());
	wifi_config_t conf_current;
	esp_wifi_get_config(WIFI_IF_AP, &conf_current);
	// Change WIFI AP default beacon interval sending to 1s.
	conf_current.ap.beacon_interval = 1000;
	drone_idfr.set_drone_id(drone_id);
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &conf_current));
	
}

/**
	* Début du code principal. C'est une boucle infinie.
*/
void loop()
{
	static uint64_t gpsMap = 0;
	switch (program) {
		case 0:
		
		// Ici on lit les données qui arrive du GPS et on les passes à la librairie TinyGPS++ pour les traiter
		while (SerialGPS.available())
		gps.encode(SerialGPS.read());
		
		// On traite le case ou le GPS a un probleme
		if (millis() > 5000 && gps.charsProcessed() < 10) {
			snprintf(buff[0], sizeof(buff[0]), "Tracker GPS");
			snprintf(buff[1], sizeof(buff[1]), "No GPS detected");
			
			if (!ssd1306_found) {
				Serial.println(buff[1]);
			}
			return;
		}
		// On traite le cas si la position GPS n'est pas valide
		if (!gps.location.isValid()) {
			if (millis() - gpsMap > 1000) {
				snprintf(buff[0], sizeof(buff[0]), "Tracker GPS");
				snprintf(buff[1], sizeof(buff[1]), "Positioning(%llu)", gpsSec++);
				
				buzz(BUZZER_PIN, 2500, 10);//tick
				
				if (!ssd1306_found) {
					Serial.println(buff[1]);
				}
				gpsMap = millis();
			}
			
			
			} else if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.course.isUpdated() && gps.speed.isUpdated()){
			// On traite le cas où la position GPS est valide.
			// On renseigne le point de démarrage quand la précision est satisfaisante
			if ( gps.satellites.value() > nb_sat ) {//+ de satellites
				buzz(BUZZER_PIN, 6000, 100);
				delay (100);
				nb_sat = gps.satellites.value();
			}
			if (!drone_idfr.has_home_set() && gps.satellites.value() > 6 && gps.hdop.hdop() < 2.0) {
				Serial.println("Setting Home Position");
				
				buzz(BUZZER_PIN, 7000, 200);
				delay (50);
				buzz(BUZZER_PIN, 7000, 200);
				delay (50);
				buzz(BUZZER_PIN, 7000, 200);
				drone_idfr.set_home_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());									
				home_alt = gps.altitude.meters();
				// Ici on ecrit sur le port Serie des données GPS pour visualisation seulement.
				if (millis() - gpsMap > 1000) {         
					Serial.print("LAT=");  Serial.print(gps.location.lat(), 6); Serial.print(" LONG="); Serial.print(gps.location.lng(), 6);
					Serial.print(" ALT=");  Serial.print(gps.altitude.meters());  Serial.print(" SAT=");  Serial.println(gps.satellites.value());
					
					gpsMap = millis();
				}												 
			}
			// On actualise les données GPS de la librairie d'identification drone.
			drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
			drone_idfr.set_heading(gps.course.deg());
			drone_idfr.set_ground_speed(gps.speed.mps());
			
			if (VMAX < gps.speed.mps()){ VMAX = gps.speed.mps();}
			
			// Ici on ecrit sur le port USB les données GPS pour visualisation seulement.
			if (millis() - gpsMap > 1000) {
				//snprintf(buff[0], sizeof(buff[0]), "UTC:%d:%d:%d", gps.time.hour(), gps.time.minute(), gps.time.second());
				snprintf(buff[0], sizeof(buff[0]), "VMAX(km/h):%.2f", float (VMAX*3.6));
				snprintf(buff[1], sizeof(buff[1]), "LNG:%.4f", gps.location.lng());
				snprintf(buff[2], sizeof(buff[2]), "LAT:%.4f", gps.location.lat());
				snprintf(buff[3], sizeof(buff[3]), "satellites:%u", gps.satellites.value());
				
				if (!ssd1306_found) {
					Serial.println(buff[0]);
					Serial.println(buff[1]);
					Serial.println(buff[2]);
					Serial.println(buff[3]);
				}
				gpsMap = millis();
			}
		}
		break;
	}
	/**
		* On regarde s'il est temps d'envoyer la trame d'identification drone : 
		*  - soit toutes les 3s,
		*  - soit si le drone s'est déplacé de 30m,
		*  - uniquement si la position Home est déjà définie,
		*  - et dans le cas où les données GPS sont nouvelles.
	*/            
	if (drone_idfr.has_home_set() && drone_idfr.time_to_send()) {
		float time_elapsed = (float(millis() - beaconSec) / 1000); 
		beaconSec = millis();
		
		/**
			* On commence par renseigner le ssid du wifi dans la trame
		*/
		// write new SSID into beacon frame
		const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
		beaconPacket[40] = ssid_size;  // set size
		memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
		const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker
		/**
			* On génère la trame wifi avec l'identfication
		*/
		const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination
		// Décommenter ce block pour voir la trame entière sur le port usb
		/*Serial.println("beaconPacket : ");
			for (auto i=0; i<sizeof(beaconPacket);i++) {
			Serial.print(beaconPacket[i], HEX);
			Serial.print(" ");
			}
		Serial.println(" ");*/
		
		/**
			* On envoie la trame
		*/
		ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, to_send, true));
		/**
			* On reset la condition d'envoi
		*/
		drone_idfr.set_last_send();
	}
	
	if (ssd1306_found) {
		if (ui.update()) {
		}
	}
}
