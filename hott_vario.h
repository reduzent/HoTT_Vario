
#define HOTT_VARIO_MSG_TEXT_LEN 21
static struct {
    uint8_t start_byte;          //#01 start uint8_t constant value 0x7c
    uint8_t vario_sensor_id;     //#02 VARIO sensort id. constat value 0x89
    uint8_t warning_beeps;       //#03 1=A 2=B ...
                                // Q    Min cell voltage sensor 1
                                // R    Min Battery 1 voltage sensor 1
                                // J    Max Battery 1 voltage sensor 1
                                // F    Min temperature sensor 1
                                // H    Max temperature sensor 1
                                // S    Min Battery voltage sensor 2
                                // K    Max Battery voltage sensor 2
                                // G    Min temperature sensor 2
                                // I    Max temperature sensor 2
                                // W    Max current
                                // V    Max capacity mAh
                                // P    Min main power voltage
                                // X    Max main power voltage
                                // O    Min altitude
                                // Z    Max altitude
                                // T    Minimum RPM
                                // Y    Maximum RPM
                                // C    m/s negative difference
                                // A    m/3s negative difference


    uint8_t sensor_id;           //#04 constant value 0x90
    uint8_t alarm_invers1;       //#05 Inverse display (alarm?) bitmask
                                //TODO: more info
    int altitude;          //#06 Altitude low uint8_t. In meters. A value of 500 means 0m
    int altitude_max;      //#08 Max. measured altitude low uint8_t. In meters. A value of 500 means 0m
    int altitude_min;      //#10 Min. measured altitude low uint8_t. In meters. A value of 500 means 0m
    int climbrate;         //#12 Climb rate in m/s. Steps of 0.01m/s. Value of 30000 = 0.00 m/s
    int climbrate3s;       //#14 Climb rate in m/3s. Steps of 0.01m/3s. Value of 30000 = 0.00 m/3s
    int climbrate10s;      //#16 Climb rate m/10s. Steps of 0.01m/10s. Value of 30000 = 0.00 m/10s
    uint8_t text_msg[HOTT_VARIO_MSG_TEXT_LEN]; //#18 Free ASCII text message
    uint8_t free_char1;          //#39 Free ASCII character.  appears right to home distance
    uint8_t free_char2;          //#40 Free ASCII character.  appears right to home direction
    uint8_t free_char3;          //#41 Free ASCII character.  appears? TODO: Check where this char appears
    uint8_t compass_direction;   //#42 Compass heading in 2� steps. 1 = 2�
    uint8_t version;             //#43 version number TODO: more info?
    uint8_t stop_byte;           //#44 stop uint8_t, constant value 0x7d
} HOTT_VARIO_MSG;
