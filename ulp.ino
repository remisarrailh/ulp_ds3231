#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "soc/rtc_io_reg.h"
#include <Wire.h>

#define DS3231_ADDR 0x68 // Adresse I2C du DS3231
#define DS3231_CONTROL_REG 0x0E // Registre de contrôle pour configurer le SQW
#define NB_IMPULSIONS 60 // 1 minute à 1 Hz

// Active le signal carré 1 Hz sur la pin SQW du DS3231
/*
DataSheet: https://www.analog.com/media/en/technical-documentation/data-sheets/DS3231.pdf
------------------------------------------------------------------------------------------
Interruption active à l'état bas ou sortie de signal carré . 
Cette broche à drain ouvert nécessite une résistance de tirage (pullup) externe 
La fonction de cette broche multifonction est déterminée par l'état du bit INTCN dans le registre de contrôle (0x0E).
Mode Signal Carré : Lorsque INTCN est réglé sur le 0, cette broche émet un signal carré dont la fréquence est déterminée par les bits RS2 et RS1.
RS2 : 0 RS1 : 0 -- Fréquence de sortie : 1 Hz
Mode Interruption : Lorsque INTCN est réglé sur le 1 logique, active une interruption à l'état bas si une alarme est déclenchée
*/
void ds3231_enable_sqw_1hz() {
  Wire.beginTransmission(DS3231_ADDR); // Adresse du DS3231
  Wire.write(DS3231_CONTROL_REG); // Sélection du registre de contrôle
  Wire.write(0x00); // INTCN=0, RS2=0, RS1=0 -> SQW 1 Hz
  Wire.endTransmission();
}

// Convertit un octet BCD (Binary-Coded Decimal) en décimal
uint8_t bcd2dec(uint8_t bcd) {
  return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Lit et affiche l'heure du DS3231
void ds3231_print_time() {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00); // Registre des secondes
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 3); // sec, min, heure
  uint8_t sec = bcd2dec(Wire.read());
  uint8_t min = bcd2dec(Wire.read());
  uint8_t hour = bcd2dec(Wire.read() & 0x3F); // Masque le bit 12/24h
  Serial.printf("Heure DS3231 : %02d:%02d:%02d\n", hour, min, sec);
}

// Lit uniquement les secondes du DS3231
uint8_t ds3231_read_seconds() {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 1);
  return bcd2dec(Wire.read());
}

// Attend que le DS3231 arrive à 00 secondes (nouvelle minute), en affichant l'heure chaque seconde
void wait_for_new_minute() {
  Serial.println("Attente de la nouvelle minute...");
  uint8_t lastSec = 0xFF; // Valeur impossible pour forcer l'affichage immédiat
  while (true) {
    uint8_t sec = ds3231_read_seconds();
    if (sec != lastSec) {
      lastSec = sec;
      ds3231_print_time();
      if (sec == 0) {
        Serial.println("Nouvelle minute atteinte !");
        break;
      }
    }
    delay(50); // Polling léger
  }
}

// Adresse en mémoire RTC où on stocke nos variables
// L'ULP et l'ESP32 partagent cette mémoire
enum {
  SLOW_PROG_ADDR = 0,
  COUNTER_ADDR = 30,      // Compteur d'impulsions
  THRESHOLD_ADDR = 31,    // Seuil de réveil
  PREV_STATE_ADDR = 32    // État précédent du GPIO (pour détection de front)
};

void setup_ulp() {

  // 2. Code ULP avec détection de front descendant (HIGH→LOW)
  // L'ULP tourne toutes les 20ms. Il compare l'état actuel du GPIO
  // avec l'état précédent stocké en mémoire pour détecter une transition.
  const ulp_insn_t program[] = {
    // -- Lire le GPIO --
    I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 8, RTC_GPIO_IN_NEXT_S + 8),  // 0: R0 = état actuel du GPIO 33 (1 ou 0)
    I_BGE(20, 1),                    // 1: Jump de 20 instructions (21) si HIGH (pas de front descendant) → Sauver HIGH et halt

    // -- Current est LOW, charger l'état précédent --
    I_MOVI(R1, PREV_STATE_ADDR),     // 2: Déplacer l'adresse de l'état précédent dans R1
    I_LD(R0, R1, 0),                 // 3: Comparer avec l'état précédent (R0 = current, R1 = prev)
    I_MOVI(R2, 0),                   // 4: Déplacer la valeur LOW dans R2
    I_ST(R2, R1, 0),                 // 5: Sauver LOW comme nouvelle état dans R1

    // -- Vérifier si front descendant (prev=HIGH) --
    I_BL(14, 1),                     // 6: Si prev LOW → pas de front → halt (PC=20)

    // -- FRONT DESCENDANT détecté ! Incrémenter --
    I_MOVI(R1, COUNTER_ADDR),        // 7: Déplacer l'adresse du compteur dans R1
    I_LD(R0, R1, 0),                 // 8: R0 = compteur
    I_ADDI(R0, R0, 1),               // 9: +1
    I_ST(R0, R1, 0),                 // 10: Sauver

    // -- Comparer au seuil --
    I_MOVI(R1, THRESHOLD_ADDR),      // 11: Déplacer l'adresse du seuil dans R1
    I_LD(R1, R1, 0),                 // 12: R1 = seuil
    I_SUBR(R0, R0, R1),              // 13: compteur - seuil
    I_BL(2, 32768),                  // 14: Si pas d'underflow (>=seuil) → wake (PC=16) Explication plus détaillé:
    I_HALT(),                         // 15: Seuil pas atteint stop (PC=20)

    // -- WAKE : reset compteur et réveiller --
    I_MOVI(R1, COUNTER_ADDR),        // 16: Adresse du compteur
    I_MOVI(R0, 0),                   // 17: Valeur 0
    I_ST(R0, R1, 0),                 // 18: Sauver 0 pour reset le compteur
    I_WAKE(),                         // 19 : Réveiller l'ESP32
    I_HALT(),                         // 20: halt générique

    // -- Sauver HIGH et halt (cible du BGE en 1) --
    I_MOVI(R1, PREV_STATE_ADDR),     // 21: Adresse de l'état précédent
    I_MOVI(R0, 1),                   // 22: Valeur HIGH
    I_ST(R0, R1, 0),                 // 23: Sauver HIGH comme nouvel état
    I_HALT(),                         // 24: halt générique
  };

  // 3. Charger et lancer
  size_t size = sizeof(program) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(SLOW_PROG_ADDR, program, &size);
  ulp_set_wakeup_period(0, 20000); // Vérifie toutes les 20ms
  ulp_run(SLOW_PROG_ADDR);
}


void diagnostic(){
      // Test diagnostic
    Serial.println("Test du signal sur GPIO 33 pendant 3 secondes...");
    int transitions = 0;
    int lastState = digitalRead(33);
    unsigned long start = millis();
    while (millis() - start < 3000) {
      int state = digitalRead(33);
      if (state != lastState) {
        transitions++;
        lastState = state;
      }
      delay(1);
    }
    Serial.printf("  Etat actuel GPIO 33 : %s\n", digitalRead(33) ? "HIGH" : "LOW");
    Serial.printf("  Transitions detectees : %d\n", transitions);
    if (transitions == 0) {
      Serial.println("  ATTENTION : Aucune transition ! Verifiez le cablage DS3231 SQW -> GPIO 33");
    } else {
      Serial.printf("  OK : Signal actif (~%d Hz)\n", transitions / 2 / 3);
    }
  }

void setup() {
  Serial.begin(115200); 
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause(); // Vérifier la cause du réveil
  Wire.begin();

  // Configurer la pin SQW (GPIO 33 / RTC_GPIO 8) - Nécessaire à refaire car l'ESP32 reset les configurations GPIO au réveil
  rtc_gpio_init(GPIO_NUM_33); // GPIO 33 est aussi RTC_GPIO 8
  rtc_gpio_set_direction(GPIO_NUM_33, RTC_GPIO_MODE_INPUT_ONLY); // Entrée uniquement
  rtc_gpio_pullup_en(GPIO_NUM_33); // Activer pull-up interne (optionnel, dépend du câblage)

  if (cause == ESP_SLEEP_WAKEUP_ULP) { // Réveil par l'ULP
    Serial.println("Reveille par l'ULP !");
    //ulp_run(SLOW_PROG_ADDR); // Réarmer le timer ULP pour le prochain cycle (pas nécessaire car toujours actif)
  } else {
    // Premier démarrage : configurer tout
    Serial.println("Premier demarrage, configuration...");
    ulp_timer_stop(); // S'assurer que l'ULP est arrêté avant de configurer
    ds3231_enable_sqw_1hz(); // Configurer le DS3231 pour émettre un signal carré 1 Hz
    diagnostic(); // Test de signal pour aider au debug
    wait_for_new_minute(); // Attendre 00 secondes avant de dormir
    
    // Préparer la mémoire et lancer l'ULP
    RTC_SLOW_MEM[THRESHOLD_ADDR] = NB_IMPULSIONS; // Seuil de 60 impulsions (1 minute à 1 Hz)
    RTC_SLOW_MEM[COUNTER_ADDR] = 0; // Initialiser le compteur à 0
    RTC_SLOW_MEM[PREV_STATE_ADDR] = digitalRead(33); // Initialiser l'état précédent avec l'état actuel du GPIO
    setup_ulp();
  }
  ds3231_print_time(); // Afficher l'heure pour vérifier si ça fonctionne
  Serial.println("Dodo...");
  esp_sleep_enable_ulp_wakeup(); // Activer le réveil par l'ULP
  esp_deep_sleep_start(); // Entrer en deep sleep (l'ULP continue de tourner)
}

void loop() {}