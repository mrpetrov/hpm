/*
* hpm.c
*
* Raspberry Pi home warm water manager which uses 1wire and GPIO.
* Plamen Petrov
*
* hpm is Plamen's custom solar controller, based on the Raspberry Pi (2,3,4).
* Data is gathered and logged every 10 seconds from 5 DS18B20 waterproof sensors,
* 4 relays are controlled via GPIO, and a GPIO pin is read to note current
* power source: grid or battery backed UPS. Commands for a counterpart system are
* sent via setting 4 designated GPIO ports output, acting as a guaranteed comms channel.
* Log data is in CSV format, to be picked up by some sort of data collection/graphing
* tool, like collectd or similar. There is also JSON file more suitable for sending data
* to data collection software like mqqt/emoncms.
* The daemon is controlled via its configuration file, which hpm can be told to
* re-read and parse while running to change config in flight. This is done by
* sending SIGUSR1 signal to the daemon process. The event is noted in the log file.
* The logfile itself can be "grep"-ed for "ALARM" and "INFO" to catch and notify
* of notable events, recorded by the daemon.
*/

#ifndef PGMVER
#error Need to define PGMVER in order to compile me!
#endif

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <time.h>

#define RUNNING_DIR     "/tmp"
#define LOCK_FILE       "/run/hpm.pid"
#define LOG_FILE        "/var/log/hpm.log"
#define DATA_FILE       "/run/shm/hpm_data.log"
#define TABLE_FILE      "/run/shm/hpm_current"
#define JSON_FILE	"/run/shm/hpm_current_json"
#define CFG_TABLE_FILE  "/run/shm/hpm_cur_cfg"
#define CONFIG_FILE     "/etc/hpm.cfg"
#define PRSSTNC_FILE      "/var/log/hpm_prsstnc"

#define BUFFER_MAX 3
#define DIRECTION_MAX 35
#define VALUE_MAX 50
#define MAXLEN 80

#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

/* Maximum difference allowed for data received from sensors between reads, C */
#define MAX_TEMP_DIFF        5

/* Number of all sensors to be used by the system */
#define TOTALSENSORS         11

/* Array of char* holding the paths to temperature DS18B20 sensors */
char* sensor_paths[TOTALSENSORS+1];

/*  var to keep track of read errors, so if a threshold is reached - the
    program can safely shut down everything, send notification and bail out;
    initialised with borderline value to trigger immediately on errors during
    start-up; the program logic tolerates 1 minute of missing sensor data
*/
unsigned short sensor_read_errors[TOTALSENSORS+1] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 };

/* current sensors temperatures - e.g. values from last read */
float sensors[TOTALSENSORS+1] = { 0, -200, -200, -200, -200, -200, -200, -200, -200, -200, -200, -200 };

/* previous sensors temperatures - e.g. values from previous to last read */
float sensors_prv[TOTALSENSORS+1] = { 0, -200, -200, -200, -200, -200, -200, -200, -200, -200, -200, -200 };

/* and sensor name mappings */
#define   Tac1cmp            sensors[1]
#define   Tac1cnd             sensors[2]
#define   The1i                 sensors[3]
#define   The1o                sensors[4]
#define   Tac2cmp            sensors[5]
#define   Tac2cnd             sensors[6]
#define   The2i                 sensors[7]
#define   The2o                sensors[8]
#define   Twi                    sensors[9]
#define   Two                   sensors[10]
#define   Tenv                  sensors[11]

#define   Tac1cmpPrev            sensors_prv[1]
#define   Tac1cndPrev             sensors_prv[2]
#define   The1iPrev                 sensors_prv[3]
#define   The1oPrev                sensors_prv[4]
#define   Tac2cmpPrev            sensors_prv[5]
#define   Tac2cndPrev             sensors_prv[6]
#define   The2iPrev                 sensors_prv[7]
#define   The2oPrev                sensors_prv[8]
#define   TwiPrev                    sensors_prv[9]
#define   TwoPrev                   sensors_prv[10]
#define   TenvPrev                  sensors_prv[11]

/* current controls state - e.g. set on last decision making */
short controls[9] = { -1, 0, 0, 0, 0, 0, 0, 0, 0 };

/* and control name mappings */
#define   Cac1cmp             controls[1]
#define   Cac1fan               controls[2]
#define   Cac1fv                 controls[3]
#define   Cac1mode            controls[4]
#define   Cac2cmp              controls[5]
#define   Cac2fan                controls[6]
#define   Cac2fv                  controls[7]
#define   Cac2mode            controls[8]

/* AC mode states:
            0 - off; 
            1 - starting; 
            2 - compressor cooling; 
            3 - fin stack heating up */

/* controls state cycles - zeroed on change to state */
unsigned long ctrlstatecycles[9] = { 1234567890, 55, 0, 0, 0, 52, 0, 0, 0 };

#define   SCac1cmp             ctrlstatecycles[1]
#define   SCac1fan               ctrlstatecycles[2]
#define   SCac1fv                 ctrlstatecycles[3]
#define   SCac1mode           ctrlstatecycles[4]
#define   SCac2cmp             ctrlstatecycles[5]
#define   SCac2fan               ctrlstatecycles[6]
#define   SCac2fv                 ctrlstatecycles[7]
#define   SCac2mode           ctrlstatecycles[8]

unsigned long C1RunCs = 0;
unsigned long C2RunCs = 0;

/* Nubmer of cycles (~5 seconds each) that the program has run */
unsigned long ProgramRunCycles  = 0;

/* timers - current hour and month vars - used in keeping things up to date */
unsigned short current_timer_hour = 0;
unsigned short current_month = 0;

/* a var to be non-zero if it is winter time - so furnace should not be allowed to go too cold */
unsigned short now_is_winter = 0;

/* Comms buffer */
unsigned short COMMS = 0;

/* If other than zero - low mode is enabled, i.e. one AC can be run */
unsigned short HPL = 0;

/* If other than zero - HIGH mode is enabled, i.e. both ACs can be run */
unsigned short HPH = 0;

/* The buffer Var that holds what needs to sent over to hwwm 
    States:
    0 == busy; no changes to state allowed/will be honered
    1 == busy; can ADD an AC
    2 == busy; can REMOVE an AC
    3 == done/request is fullfilled; can change state as desired */
unsigned short sendBits = 0;

struct cfg_struct
{
    char    ac1cmp_sensor[MAXLEN];
    char    ac1cnd_sensor[MAXLEN];
    char    he1i_sensor[MAXLEN];
    char    he1o_sensor[MAXLEN];
    char    ac2cmp_sensor[MAXLEN];
    char    ac2cnd_sensor[MAXLEN];
    char    he2i_sensor[MAXLEN];
    char    he2o_sensor[MAXLEN];
    char    wi_sensor[MAXLEN];
    char    wo_sensor[MAXLEN];
    char    tenv_sensor[MAXLEN];
    char    invert_output_str[MAXLEN];
    int      invert_output;
    char    ac1cmp_pin_str[MAXLEN];
    int     ac1cmp_pin;
    char    ac1fan_pin_str[MAXLEN];
    int     ac1fan_pin;
    char    ac1v_pin_str[MAXLEN];
    int     ac1v_pin;
    char    ac2cmp_pin_str[MAXLEN];
    int     ac2cmp_pin;
    char    ac2fan_pin_str[MAXLEN];
    int     ac2fan_pin;
    char    ac2v_pin_str[MAXLEN];
    int     ac2v_pin;
    char    commspin1_pin_str[MAXLEN];
    int     commspin1_pin;
    char    commspin2_pin_str[MAXLEN];
    int     commspin2_pin;
    char    commspin3_pin_str[MAXLEN];
    int     commspin3_pin;
    char    commspin4_pin_str[MAXLEN];
    int     commspin4_pin;
    char    mode_str[MAXLEN];
    int     mode;
    char    use_ac1_str[MAXLEN];
    int     use_ac1;
    char    use_ac2_str[MAXLEN];
    int     use_ac2;
    char    wicorr_str[MAXLEN];
    float  wicorr;
    char    wocorr_str[MAXLEN];
    float  wocorr;
    char    tenvcorr_str[MAXLEN];
    float  tenvcorr;
}
cfg_struct;

struct cfg_struct cfg;

short need_to_read_cfg = 0;

short just_started = 0;

/* FORWARD DECLARATIONS so functions can be used in preceding ones */
short
DisableGPIOpins();
/* end of forward-declared functions */

void
rangecheck_GPIO_pin( int p )
{
    if (p < 4) p = 4;
    if (p > 27) p = 27;
}

short
not_every_GPIO_pin_is_UNIQUE()
{
	short result=0;
	if (cfg.ac1cmp_pin == cfg.ac1fan_pin) result++;
	if (cfg.ac1cmp_pin == cfg.ac1v_pin) result++;
	if (cfg.ac1cmp_pin == cfg.ac2cmp_pin) result++;
	if (cfg.ac1cmp_pin == cfg.ac2fan_pin) result++;
	if (cfg.ac1cmp_pin == cfg.ac2v_pin) result++;
	if (cfg.ac1cmp_pin == cfg.commspin1_pin) result++;
	if (cfg.ac1cmp_pin == cfg.commspin2_pin) result++;
	if (cfg.ac1cmp_pin == cfg.commspin3_pin) result++;
	if (cfg.ac1cmp_pin == cfg.commspin4_pin) result++;
	if (cfg.ac1fan_pin == cfg.ac1v_pin) result++;
	if (cfg.ac1fan_pin == cfg.ac2cmp_pin) result++;
	if (cfg.ac1fan_pin == cfg.ac2fan_pin) result++;
	if (cfg.ac1fan_pin == cfg.ac2v_pin) result++;
	if (cfg.ac1fan_pin == cfg.commspin1_pin) result++;
	if (cfg.ac1fan_pin == cfg.commspin2_pin) result++;
	if (cfg.ac1fan_pin == cfg.commspin3_pin) result++;
	if (cfg.ac1fan_pin == cfg.commspin4_pin) result++;
	if (cfg.ac1v_pin == cfg.ac2cmp_pin) result++;
	if (cfg.ac1v_pin == cfg.ac2fan_pin) result++;
	if (cfg.ac1v_pin == cfg.ac2v_pin) result++;
	if (cfg.ac1v_pin == cfg.commspin1_pin) result++;
	if (cfg.ac1v_pin == cfg.commspin2_pin) result++;
	if (cfg.ac1v_pin == cfg.commspin3_pin) result++;
	if (cfg.ac1v_pin == cfg.commspin4_pin) result++;
	if (cfg.ac2cmp_pin == cfg.ac2fan_pin) result++;
	if (cfg.ac2cmp_pin == cfg.ac2v_pin) result++;
	if (cfg.ac2cmp_pin == cfg.commspin1_pin) result++;
	if (cfg.ac2cmp_pin == cfg.commspin2_pin) result++;
	if (cfg.ac2cmp_pin == cfg.commspin3_pin) result++;
	if (cfg.ac2cmp_pin == cfg.commspin4_pin) result++;
	if (cfg.ac2fan_pin == cfg.ac2v_pin) result++;
	if (cfg.ac2fan_pin == cfg.commspin1_pin) result++;
	if (cfg.ac2fan_pin == cfg.commspin2_pin) result++;
	if (cfg.ac2fan_pin == cfg.commspin3_pin) result++;
	if (cfg.ac2fan_pin == cfg.commspin4_pin) result++;
	if (cfg.ac2v_pin == cfg.commspin1_pin) result++;
	if (cfg.ac2v_pin == cfg.commspin2_pin) result++;
	if (cfg.ac2v_pin == cfg.commspin3_pin) result++;
	if (cfg.ac2v_pin == cfg.commspin4_pin) result++;
	if (cfg.commspin1_pin == cfg.commspin2_pin) result++;
	if (cfg.commspin1_pin == cfg.commspin3_pin) result++;
	if (cfg.commspin1_pin == cfg.commspin4_pin) result++;
	if (cfg.commspin2_pin == cfg.commspin3_pin) result++;
	if (cfg.commspin2_pin == cfg.commspin4_pin) result++;
	if (cfg.commspin3_pin == cfg.commspin4_pin) result++;
	return result;
}

void
rangecheck_mode( int m )
{
    if (m < 0) m = 0;
    if (m > 8) m = 0;
}

void
SetDefaultPINs() {
    cfg.ac1cmp_pin = 5;
    cfg.ac1fan_pin = 6;
    cfg.ac1v_pin = 13;
    cfg.ac1cmp_pin = 16;
    cfg.ac1fan_pin = 19;
    cfg.ac1v_pin = 20;
    cfg.commspin1_pin = 17;
    cfg.commspin2_pin = 18;
    cfg.commspin3_pin = 27;
    cfg.commspin4_pin = 22;
}

void
SetDefaultCfg() {
    strcpy( cfg.ac1cmp_sensor, "/dev/zero/1");
    strcpy( cfg.ac1cnd_sensor, "/dev/zero/2");
    strcpy( cfg.he1i_sensor, "/dev/zero/3");
    strcpy( cfg.he1o_sensor, "/dev/zero/4");
    strcpy( cfg.ac2cmp_sensor, "/dev/zero/5");
    strcpy( cfg.ac2cnd_sensor, "/dev/zero/6");
    strcpy( cfg.he2i_sensor, "/dev/zero/7");
    strcpy( cfg.he2o_sensor, "/dev/zero/8");
    strcpy( cfg.wi_sensor, "/dev/zero/9");
    strcpy( cfg.wo_sensor, "/dev/zero/10");
    strcpy( cfg.tenv_sensor, "/dev/zero/11");
    SetDefaultPINs();
    cfg.invert_output = 1;
    cfg.mode = 1;
    cfg.use_ac1 = 1;
    cfg.use_ac2 = 1;
    cfg.wicorr = 0;
    cfg.wocorr = 0;
    cfg.tenvcorr = 0;

    sensor_paths[0] = (char *) &cfg.ac1cmp_sensor;
    sensor_paths[1] = (char *) &cfg.ac1cmp_sensor;
    sensor_paths[2] = (char *) &cfg.ac1cnd_sensor;
    sensor_paths[3] = (char *) &cfg.he1i_sensor;
    sensor_paths[4] = (char *) &cfg.he1o_sensor;
    sensor_paths[5] = (char *) &cfg.ac2cmp_sensor;
    sensor_paths[6] = (char *) &cfg.ac2cnd_sensor;
    sensor_paths[7] = (char *) &cfg.he2i_sensor;
    sensor_paths[8] = (char *) &cfg.he2o_sensor;
    sensor_paths[9] = (char *) &cfg.wi_sensor;
    sensor_paths[10] = (char *) &cfg.wo_sensor;
    sensor_paths[11] = (char *) &cfg.tenv_sensor;
}

short
log_message(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );
    sprintf( file_string, "%s %s", timestamp, message );
    logfile = fopen( filename, "a" );
    if ( !logfile ) return -1;
    fprintf( logfile, "%s\n", file_string );
    fclose( logfile );
    return 0;
}

/* this version of the logging function destroys the opened file contents */
void
log_msg_ovr(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );
    sprintf( file_string, "%s%s", timestamp, message );
    logfile = fopen( filename, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "%s\n", file_string );
    fclose( logfile );
}

/* this version of the logging function destroys the opened file contents, no timestamp and new line */
void
log_msg_cln(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];

    sprintf( file_string, "%s", message );
    logfile = fopen( filename, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "%s", file_string );
    fclose( logfile );
}

/* trim: get rid of trailing and leading whitespace...
    ...including the annoying "\n" from fgets()
*/
char *
trim (char * s)
{
    /* Initialize start, end pointers */
    char *s1 = s, *s2 = &s[strlen (s) - 1];

    /* Trim and delimit right side */
    while ( (isspace (*s2)) && (s2 >= s1) )
    s2--;
    *(s2+1) = '\0';

    /* Trim left side */
    while ( (isspace (*s1)) && (s1 < s2) )
    s1++;

    /* Copy finished string */
    strcpy (s, s1);
    return s;
}

void
parse_config()
{
    int i = 0;
    float f = 0;
    char *s, buff[150];
    FILE *fp = fopen(CONFIG_FILE, "r");
    if (fp == NULL) {
        log_message(LOG_FILE,"WARNING: Failed to open "CONFIG_FILE" file for reading!");
        } else {
        /* Read next line */
        while ((s = fgets (buff, sizeof buff, fp)) != NULL)
        {
            /* Skip blank lines and comments */
            if (buff[0] == '\n' || buff[0] == '#')
            continue;

            /* Parse name/value pair from line */
            char name[MAXLEN], value[MAXLEN];
            s = strtok (buff, "=");
            if (s==NULL) continue;
            else strncpy (name, s, MAXLEN);
            s = strtok (NULL, "=");
            if (s==NULL) continue;
            else strncpy (value, s, MAXLEN);
            trim (value);

            /* Copy into correct entry in parameters struct */
            if (strcmp(name, "ac1cmp_sensor")==0)
            strncpy (cfg.ac1cmp_sensor, value, MAXLEN);
            else if (strcmp(name, "ac1cnd_sensor")==0)
            strncpy (cfg.ac1cnd_sensor, value, MAXLEN);
            else if (strcmp(name, "he1i_sensor")==0)
            strncpy (cfg.he1i_sensor, value, MAXLEN);
            else if (strcmp(name, "he1o_sensor")==0)
            strncpy (cfg.he1o_sensor, value, MAXLEN);
            else if (strcmp(name, "ac2cmp_sensor")==0)
            strncpy (cfg.ac2cmp_sensor, value, MAXLEN);
            else if (strcmp(name, "ac2cnd_sensor")==0)
            strncpy (cfg.ac2cnd_sensor, value, MAXLEN);
            else if (strcmp(name, "he2i_sensor")==0)
            strncpy (cfg.he2i_sensor, value, MAXLEN);
            else if (strcmp(name, "he2o_sensor")==0)
            strncpy (cfg.he2o_sensor, value, MAXLEN);
            else if (strcmp(name, "wi_sensor")==0)
            strncpy (cfg.wi_sensor, value, MAXLEN);
            else if (strcmp(name, "wo_sensor")==0)
            strncpy (cfg.wo_sensor, value, MAXLEN);
            else if (strcmp(name, "tenv_sensor")==0)
            strncpy (cfg.tenv_sensor, value, MAXLEN);
            else if (strcmp(name, "ac1cmp_pin")==0)
            strncpy (cfg.ac1cmp_pin_str, value, MAXLEN);
            else if (strcmp(name, "ac1fan_pin")==0)
            strncpy (cfg.ac1fan_pin_str, value, MAXLEN);
            else if (strcmp(name, "ac1v_pin")==0)
            strncpy (cfg.ac1v_pin_str, value, MAXLEN);
            else if (strcmp(name, "ac2cmp_pin")==0)
            strncpy (cfg.ac2cmp_pin_str, value, MAXLEN);
            else if (strcmp(name, "ac2fan_pin")==0)
            strncpy (cfg.ac2fan_pin_str, value, MAXLEN);
            else if (strcmp(name, "ac2v_pin")==0)
            strncpy (cfg.ac2v_pin_str, value, MAXLEN);
            else if (strcmp(name, "commspin1_pin")==0)
            strncpy (cfg.commspin1_pin_str, value, MAXLEN);
            else if (strcmp(name, "commspin2_pin")==0)
            strncpy (cfg.commspin2_pin_str, value, MAXLEN);
            else if (strcmp(name, "commspin3_pin")==0)
            strncpy (cfg.commspin3_pin_str, value, MAXLEN);
            else if (strcmp(name, "commspin4_pin")==0)
            strncpy (cfg.commspin4_pin_str, value, MAXLEN);
            else if (strcmp(name, "invert_output")==0)
            strncpy (cfg.invert_output_str, value, MAXLEN);
            else if (strcmp(name, "mode")==0)
            strncpy (cfg.mode_str, value, MAXLEN);
            else if (strcmp(name, "use_ac1")==0)
            strncpy (cfg.use_ac1_str, value, MAXLEN);
            else if (strcmp(name, "use_ac2")==0)
            strncpy (cfg.use_ac2_str, value, MAXLEN);
            else if (strcmp(name, "wicorr")==0)
            strncpy (cfg.wicorr_str, value, MAXLEN);
            else if (strcmp(name, "wocorr")==0)
            strncpy (cfg.wocorr_str, value, MAXLEN);
            else if (strcmp(name, "tenvcorr")==0)
            strncpy (cfg.tenvcorr_str, value, MAXLEN);
        }
        /* Close file */
        fclose (fp);
    }

    /* Convert strings to int */
    strcpy( buff, cfg.ac1cmp_pin_str );
    i = atoi( buff );
    cfg.ac1cmp_pin = i;
    rangecheck_GPIO_pin( cfg.ac1cmp_pin );
    strcpy( buff, cfg.ac1fan_pin_str );
    i = atoi( buff );
    cfg.ac1fan_pin = i;
    rangecheck_GPIO_pin( cfg.ac1fan_pin );
    strcpy( buff, cfg.ac1v_pin_str );
    i = atoi( buff );
    cfg.ac1v_pin = i;
    rangecheck_GPIO_pin( cfg.ac1v_pin );
    strcpy( buff, cfg.ac2cmp_pin_str );
    i = atoi( buff );
    cfg.ac2cmp_pin = i;
    rangecheck_GPIO_pin( cfg.ac2cmp_pin );
    strcpy( buff, cfg.ac2fan_pin_str );
    i = atoi( buff );
    cfg.ac2fan_pin = i;
    rangecheck_GPIO_pin( cfg.ac2fan_pin );
    strcpy( buff, cfg.ac2v_pin_str );
    i = atoi( buff );
    cfg.ac2v_pin = i;
    rangecheck_GPIO_pin( cfg.ac2v_pin );
    strcpy( buff, cfg.commspin1_pin_str );
    i = atoi( buff );
    cfg.commspin1_pin = i;
    rangecheck_GPIO_pin( cfg.commspin1_pin );
    strcpy( buff, cfg.commspin2_pin_str );
    i = atoi( buff );
    cfg.commspin2_pin = i;
    rangecheck_GPIO_pin( cfg.commspin2_pin );
    strcpy( buff, cfg.commspin3_pin_str );
    i = atoi( buff );
    cfg.commspin3_pin = i;
    rangecheck_GPIO_pin( cfg.commspin3_pin );
    strcpy( buff, cfg.commspin4_pin_str );
    i = atoi( buff );
    cfg.commspin4_pin = i;
    rangecheck_GPIO_pin( cfg.commspin4_pin );
	if (not_every_GPIO_pin_is_UNIQUE()) {
       log_message(LOG_FILE,"ALERT: Check config - found configured GPIO pin assigned more than once!");
       log_message(LOG_FILE,"ALERT: The above is an error. Switching to using default GPIO pins config...");
       SetDefaultPINs();
	}
    strcpy( buff, cfg.invert_output_str );
    i = atoi( buff );
    cfg.invert_output = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */

    strcpy( buff, cfg.mode_str );
    i = atoi( buff );
    cfg.mode = i;
    rangecheck_mode( cfg.mode );
    strcpy( buff, cfg.use_ac1_str );
    i = atoi( buff );
    cfg.use_ac1 = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.use_ac2_str );
    i = atoi( buff );
    cfg.use_ac2 = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.wicorr_str );
    f = atof( buff );
    cfg.wicorr = f;
    strcpy( buff, cfg.wocorr_str );
    f = atof( buff );
    cfg.wocorr = f;
    strcpy( buff, cfg.tenvcorr_str );
    f = atof( buff );
    cfg.tenvcorr = f;

    /* Prepare log messages with sensor paths and write them to log file */
    sprintf( buff, "AC1 compressor temp sensor file: %s", cfg.ac1cmp_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "AC1 condenser temp sensor file: %s", cfg.ac1cnd_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Heat exchanger 1 IN temp sensor file: %s", cfg.he1i_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Heat exchanger 1 OUT temp sensor file: %s", cfg.he1o_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "AC2 compressor temp sensor file: %s", cfg.ac2cmp_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "AC2 condenser temp sensor file: %s", cfg.ac2cnd_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Heat exchanger 2 IN temp sensor file: %s", cfg.he2i_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Heat exchanger 2 OUT temp sensor file: %s", cfg.he2o_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Water IN temp sensor file: %s", cfg.wi_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Water OUT temp sensor file: %s", cfg.wo_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Outdoor environment temp sensor file: %s", cfg.tenv_sensor );
    log_message(LOG_FILE, buff);
    /* Prepare log messages with GPIO pins used and write them to log file */
    sprintf( buff, "Using COMMs GPIO pins (BCM mode) as follows: comms1: %d, comms2: %d, comms3: %d, "\
	"comms4: %d ", cfg.commspin1_pin, cfg.commspin2_pin, cfg.commspin3_pin, cfg.commspin4_pin );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Using OUTPUT GPIO pins (BCM mode) as follows: AC1 comp: %d, AC1 fan: %d, AC1 valve: %d",
           cfg.ac1cmp_pin, cfg.ac1fan_pin, cfg.ac1v_pin );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Using OUTPUT GPIO pins (BCM mode) as follows: AC2 comp: %d, AC2 fan: %d, AC2 valve: %d",
           cfg.ac2cmp_pin, cfg.ac2fan_pin, cfg.ac2v_pin );
    log_message(LOG_FILE, buff);
    if (cfg.invert_output) {
        sprintf( buff, "OUTPUT GPIO pins controlling is INVERTED - ON is LOW (0)" );
        log_message(LOG_FILE, buff);
    }
    else {
        sprintf( buff, "OUTPUT GPIO pins controlling is STRAIGHT - ON is HIGH (1)" );
        log_message(LOG_FILE, buff);
    }
    /* Prepare log message part 1 and write it to log file */
    if (fp == NULL) {
        sprintf( buff, "INFO: Using values: Mode=%d, use AC1=%d, use AC2=%d, corrections: water IN=%5.3f, water OUT=%5.3f, Tenv=%5.3f",
            cfg.mode, cfg.use_ac1, cfg.use_ac2, cfg.wicorr, cfg.wocorr, cfg.tenvcorr );
        } else {
        sprintf( buff, "INFO: Read CFG file: Mode=%d, use AC1=%d, use AC2=%d, corrections: water IN=%5.3f, water OUT=%5.3f, Tenv=%5.3f",
            cfg.mode, cfg.use_ac1, cfg.use_ac2, cfg.wicorr, cfg.wocorr, cfg.tenvcorr );
    }
    log_message(LOG_FILE, buff);
    if(!cfg.mode){
        sprintf( buff, "WARNING: For some reason, hpm is configured to be OFF, i.e. the config file option \"mode\" is recognised as ZERO !!!!!" );
        log_message(LOG_FILE, buff);
    }
}

void
WritePersistentData() {
    FILE *logfile;
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );

    logfile = fopen( PRSSTNC_FILE, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "# hwwm data persistence file written %s\n", timestamp );
    fprintf( logfile, "C1RunCs=%ld\n", C1RunCs );
    fprintf( logfile, "C2RunCs=%ld\n", C2RunCs );
    fclose( logfile );
}

void
ReadPersistentData() {
    unsigned long tmp = 0;
    char *s, buff[150];
    char C1RunCs_str[MAXLEN];
    char C2RunCs_str[MAXLEN];
    short should_write=0;
    strcpy( C1RunCs_str, "0" );
    strcpy( C2RunCs_str, "0" );
    FILE *fp = fopen(PRSSTNC_FILE, "r");
    if (fp == NULL) {
        log_message(LOG_FILE,"WARNING: Failed to open "PRSSTNC_FILE" file for reading!");
        should_write = 1;
        } else {
        /* Read next line */
        while ((s = fgets (buff, sizeof buff, fp)) != NULL)
        {
            /* Skip blank lines and comments */
            if (buff[0] == '\n' || buff[0] == '#')
            continue;

            /* Parse name/value pair from line */
            char name[MAXLEN], value[MAXLEN];
            s = strtok (buff, "=");
            if (s==NULL) continue;
            else strncpy (name, s, MAXLEN);
            s = strtok (NULL, "=");
            if (s==NULL) continue;
            else strncpy (value, s, MAXLEN);
            trim (value);

            /* Copy data in corresponding strings */
            if (strcmp(name, "C1RunCs")==0)
            strncpy (C1RunCs_str, value, MAXLEN);
            else if (strcmp(name, "C2RunCs")==0)
            strncpy (C2RunCs_str, value, MAXLEN);
        }
        /* Close file */
        fclose (fp);
    }

    if (should_write) {
        log_message(LOG_FILE, "INFO: Creating missing persistent data file...");
        WritePersistentData();
    }
    else {
        /* Convert strings to float */
        strcpy( buff, C1RunCs_str );
        tmp = atol( buff );
        C1RunCs = tmp;
        strcpy( buff, C2RunCs_str );
        tmp = atol( buff );
        C2RunCs = tmp;
    }

    /* Prepare log message and write it to log file */
    if (fp == NULL) {
        sprintf( buff, "INFO: Using compressor run cycles start values: C1RunCs=%ld, C2RunCs=%ld",
        C1RunCs, C2RunCs );
        } else {
        sprintf( buff, "INFO: Read compressor run cycles start values: C1RunCs=%ld, C2RunCs=%ld",
        C1RunCs, C2RunCs );
    }
    log_message(LOG_FILE, buff);
}

int
GPIOExport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO export for writing!");
        return(-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return(0);
}

int
GPIOUnexport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO unexport for writing!");
        return(-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return(0);
}

int
GPIODirection(int pin, int dir)
{
    static const char s_directions_str[]  = "in\0out";

    char path[DIRECTION_MAX];
    int fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO direction for writing!");
        return(-1);
    }

    if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
        log_message(LOG_FILE,"Failed to set GPIO direction!");
        return(-1);
    }

    close(fd);
    return(0);
}

int
GPIORead(int pin)
{
    char path[VALUE_MAX];
    char value_str[3];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO value for reading!");
        return(-1);
    }

    if (-1 == read(fd, value_str, 3)) {
        log_message(LOG_FILE,"Failed to read GPIO value!");
        return(-1);
    }

    close(fd);

    return(atoi(value_str));
}

int
GPIOWrite(int pin, int value)
{
    static const char s_values_str[] = "01";

    char path[VALUE_MAX];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO value for writing!");
        return(-1);
    }

    if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
        log_message(LOG_FILE,"Failed to write GPIO value!");
        return(-1);
    }

    close(fd);
    return(0);
}

/*
    Example output of a sensor file:

    pi@raspberrypi ~ $ time cat  /sys/bus/w1/devices/28-0301a279c3c3/w1_slave
    29 01 55 05 7f a5 a5 66 b3 : crc=b3 YES
    29 01 55 05 7f a5 a5 66 b3 t=18562

    real    0m0.896s
    user    0m0.002s
    sys     0m0.027s
*/

float
sensorRead(const char* sensor)
{
    char path[VALUE_MAX];
    char value_str[50];
    int fd;
    char *str = "84 01 55 00 3f ff 3f 10 d7 t=114250";
    const char *result = str;
    long int_temp = 0;
    float temp = -200;
    /* if having trouble - return -200 */

    /* try to open sensor file */
    snprintf(path, VALUE_MAX, "%s", sensor);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Error opening sensor file. Continuing.");
        return(temp);
    }

    /* read the first line of data */
    if (-1 == read(fd, value_str, 39)) {
        log_message(LOG_FILE,"Error reading from sensor file. Continuing.");
        close(fd);
        return(temp);
    }

    /* throw the first line away */
    strncpy(value_str, " ", 48);

    /* read the second line into value_str */
    if (-1 == read(fd, value_str, 35)) {
        log_message(LOG_FILE,"Error reading row 2 from sensor file. Continuing.");
        close(fd);
        return(temp);
    }

    /* close the file - we are done with it */
    close(fd);

    /* transform sensor data to float */
    if((result = strchr((char *)&value_str, '=')) != NULL) {
        /* increment result to avoid the '=' */
        ++result;
        int_temp = atol( result );
        temp = ((float)int_temp) / 1000;
    }

    /* return the read temperature */
    return(temp);
}

void
signal_handler(int sig)
{
    switch(sig) {
        case SIGUSR1:
        log_message(LOG_FILE, "INFO: Signal SIGUSR1 caught. Will re-read config file soon. *************************");
        need_to_read_cfg = 1;
        break;
        case SIGUSR2:
        log_message(LOG_FILE, "INFO: Signal SIGUSR2 caught. Not implemented. Continuing. *************************");
        break;
        case SIGHUP:
        log_message(LOG_FILE, "INFO: Signal SIGHUP caught. Not implemented. Continuing. *************************");
        break;
        case SIGTERM:
        log_message(LOG_FILE, "INFO: Terminate signal caught. Stopping. *************************");
        if ( ! DisableGPIOpins() ) {
            log_message(LOG_FILE, "WARNING: Errors disabling GPIO pins! Quitting anyway. *************************");
            exit(14);
        }
        // this run was ProgramRunCycles cycles ;) 
        log_message(LOG_FILE,"Exiting normally. Bye, bye! *************************");
        exit(0);
        break;
    }
}

void
daemonize()
{
    int i, lfp;
    char str[10];

    if(getppid()==1) return; /* already a daemon */
    i=fork();
    if (i<0) { printf("hpm daemonize(): Fork error!\n"); exit(1); }/* fork error */
    if (i>0) exit(0); /* parent exits */
    /* child (daemon) continues */
    setsid(); /* obtain a new process group */
    for (i=getdtablesize();i>=0;--i) close(i); /* close all descriptors */
    i=open("/dev/null",O_RDWR); dup(i); dup(i); /* handle standart I/O */
    umask(022); /* set newly created file permissions */
    chdir(RUNNING_DIR); /* change running directory */
    lfp=open(LOCK_FILE,O_RDWR|O_CREAT,0644);
    if (lfp<0) exit(2); /* can not open */
    if (lockf(lfp,F_TLOCK,0)<0) exit(0); /* can not lock */
    /* first instance continues */
    sprintf(str,"%d\n",getpid());
    write(lfp,str,strlen(str)); /* record pid to lockfile */
    signal(SIGCHLD,SIG_IGN); /* ignore child */
    signal(SIGTSTP,SIG_IGN); /* ignore tty signals */
    signal(SIGTTOU,SIG_IGN);
    signal(SIGTTIN,SIG_IGN);
    signal(SIGUSR1,signal_handler); /* catch signal USR1 */
    signal(SIGUSR2,signal_handler); /* catch signal USR2 */
    signal(SIGHUP,signal_handler); /* catch hangup signal */
    signal(SIGTERM,signal_handler); /* catch kill signal */
}

/* the following 3 functions RETURN 0 ON ERROR! (its to make the program nice to read) */
short
EnableGPIOpins()
{
    if (-1 == GPIOExport(cfg.ac1cmp_pin)) return 0;
    if (-1 == GPIOExport(cfg.ac1fan_pin)) return 0;
    if (-1 == GPIOExport(cfg.ac1v_pin)) return 0;
    if (-1 == GPIOExport(cfg.ac2cmp_pin)) return 0;
    if (-1 == GPIOExport(cfg.ac2fan_pin)) return 0;
    if (-1 == GPIOExport(cfg.ac2v_pin)) return 0;
    if (-1 == GPIOExport(cfg.commspin1_pin)) return 0;
    if (-1 == GPIOExport(cfg.commspin2_pin)) return 0;
    if (-1 == GPIOExport(cfg.commspin3_pin)) return 0;
    if (-1 == GPIOExport(cfg.commspin4_pin)) return 0;
    return -1;
}

short
SetGPIODirection()
{
    /* input pins */
    if (-1 == GPIODirection(cfg.commspin1_pin, IN))  return 0;
    if (-1 == GPIODirection(cfg.commspin2_pin, IN))  return 0;
    /* output pins */
    if (-1 == GPIODirection(cfg.ac1cmp_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.ac1fan_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.ac1v_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.ac2cmp_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.ac2fan_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.ac2v_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.commspin3_pin, OUT))  return 0;
    if (-1 == GPIODirection(cfg.commspin4_pin, OUT))  return 0;
    return -1;
}

short
DisableGPIOpins()
{
    if (-1 == GPIOUnexport(cfg.ac1cmp_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.ac1fan_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.ac1v_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.ac2cmp_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.ac2fan_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.ac2v_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.commspin1_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.commspin2_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.commspin3_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.commspin4_pin)) return 0;
    return -1;
}

void
ReadSensors() {
    float new_val = 0;
    int i;
    char msg[100];

    for (i=1;i<=TOTALSENSORS;i++) {
        new_val = sensorRead(sensor_paths[i]);
        if ( new_val != -200 ) {
            if (sensor_read_errors[i]) sensor_read_errors[i]--;
            if (just_started) { sensors_prv[i] = new_val; sensors[i] = new_val; }
            if (new_val < (sensors_prv[i]-MAX_TEMP_DIFF)) {
                sprintf( msg, "WARNING: Correcting LOW %6.3f for sensor %d with %6.3f.", new_val, i, sensors_prv[i]-MAX_TEMP_DIFF );
                log_message(LOG_FILE, msg);
                new_val = sensors_prv[i]-MAX_TEMP_DIFF;
            }
            if (new_val > (sensors_prv[i]+MAX_TEMP_DIFF)) {
                sprintf( msg, "WARNING: Correcting HIGH %6.3f for sensor %d with %6.3f.", new_val, i, sensors_prv[i]+MAX_TEMP_DIFF );
                log_message(LOG_FILE, msg);
                new_val = sensors_prv[i]+MAX_TEMP_DIFF;
            }
            sensors_prv[i] = sensors[i];
            sensors[i] = new_val;
        }
        else {
            sensor_read_errors[i]++;
            sprintf( msg, "WARNING: Sensor %d ReadSensors() errors++. Counter at %d.", i, sensor_read_errors[i] );
            log_message(LOG_FILE, msg);
        }
    }
    /* Allow for maximum of 4 consecutive 5 seconds intervals of missing sensor data
    on any of the sensors before quitting screaming... */
    for (i=1;i<=TOTALSENSORS;i++) {
        if (sensor_read_errors[i]>4) {
            /* log the errors, clean up and bail out */
            log_message(LOG_FILE, "ALARM: Too many sensor read errors! Stopping.");
            if ( ! DisableGPIOpins() ) {
                log_message(LOG_FILE, "ALARM: GPIO disable failed on handling sensor read failures.");
                exit(66);
            }
            exit(55);
        }
    }
}

/* Read comms and assemble the global byte COMMS, as sent by hwwm */
void
ReadCommsPins() {
    unsigned short temp = 0;
    HPL = 0;
    HPH = 0;
    COMMS = 0;
    temp = GPIORead(cfg.commspin1_pin);
    if (temp) COMMS |= 1;
    temp = GPIORead(cfg.commspin2_pin);
    if (temp) COMMS |= 2;
    if (COMMS==1) HPL = 1;
    if (COMMS==2) HPH = 1;
}

/* Write comms  */
void
WriteCommsPins() {
    GPIOWrite( cfg.commspin3_pin,  (sendBits&1) );
    GPIOWrite( cfg.commspin4_pin,  (sendBits&2) );
}

/* Function to make GPIO state represent what is in controls[] */
void
ControlStateToGPIO() {
    /* put state on GPIO pins */
    if (cfg.invert_output) {
            GPIOWrite( cfg.ac1cmp_pin, !Cac1cmp );
            GPIOWrite( cfg.ac1fan_pin, !Cac1fan );
            GPIOWrite( cfg.ac1v_pin, !Cac1fv );
            GPIOWrite( cfg.ac2cmp_pin, !Cac2cmp );
            GPIOWrite( cfg.ac2fan_pin, !Cac2fan );
            GPIOWrite( cfg.ac2v_pin, !Cac2fv );
    }
    else {
            GPIOWrite( cfg.ac1cmp_pin, Cac1cmp );
            GPIOWrite( cfg.ac1fan_pin, Cac1fan );
            GPIOWrite( cfg.ac1v_pin, Cac1fv );
            GPIOWrite( cfg.ac2cmp_pin, Cac2cmp );
            GPIOWrite( cfg.ac2fan_pin, Cac2fan );
            GPIOWrite( cfg.ac2v_pin, Cac2fv );
    }
}

void
write_log_start() {
    log_message(LOG_FILE,"INFO: hpm "PGMVER" now starting up...");
    log_message(LOG_FILE,"Running in "RUNNING_DIR", config file "CONFIG_FILE );
    log_message(LOG_FILE,"PID written to "LOCK_FILE", writing CSV data to "DATA_FILE );
    log_message(LOG_FILE,"Writing table data for collectd to "TABLE_FILE );
    log_message(LOG_FILE,"Persistent data file is "PRSSTNC_FILE );
}

void
LogData(short _ST_L) {
    static char data[280];
    unsigned short diff=0;
    unsigned short RS=0; /* real state */
    if (Cac1cmp) RS|=1;
    if (Cac1fan) RS|=2;
    if (Cac1fv) RS|=4;
    if (Cac2cmp) RS|=8;
    if (Cac2fan) RS|=16;
    if (Cac2fv) RS|=32;
    diff = (_ST_L ^ RS);

    sprintf( data, "AC1: %4.1f,%4.1f,%4.1f,%4.1f;  AC2:%4.1f,%4.1f,%4.1f,%4.1f;  %6.3f,%6.3f,%6.3f ",
    Tac1cmp, Tac1cnd, The1i, The1o, Tac2cmp, Tac2cnd, The2i, The2o, Twi, Two, Tenv );
    if (Cac1mode==0) sprintf( data + strlen(data), "M1: off   ");
    if (Cac1mode==1) sprintf( data + strlen(data), "M1:starting");
    if (Cac1mode==2) sprintf( data + strlen(data), "M1:c cooling");
    if (Cac1mode==3) sprintf( data + strlen(data), "M1:fins heat");
    if (Cac1mode==4) sprintf( data + strlen(data), "M1:defrost");
    if (Cac1mode==5) sprintf( data + strlen(data), "M1:off (OHP)");
    if (Cac2mode==0) sprintf( data + strlen(data), " M2: off   ");
    if (Cac2mode==1) sprintf( data + strlen(data), " M2:starting");
    if (Cac2mode==2) sprintf( data + strlen(data), " M2:c cooling");
    if (Cac2mode==3) sprintf( data + strlen(data), " M2:fins heat");
    if (Cac2mode==4) sprintf( data + strlen(data), " M2:defrost");
    if (Cac2mode==5) sprintf( data + strlen(data), " M2:off (OHP)");
    if (_ST_L) {
        sprintf( data + strlen(data), "  WANTED:");
        if (_ST_L&1) sprintf( data + strlen(data), " C1");
        if (_ST_L&2) sprintf( data + strlen(data), " F1");
        if (_ST_L&4) sprintf( data + strlen(data), " V1");
        if (_ST_L&8) sprintf( data + strlen(data), " C2");
        if (_ST_L&16) sprintf( data + strlen(data), " F2");
        if (_ST_L&32) sprintf( data + strlen(data), " V2");
    } else sprintf( data + strlen(data), "   idle    ");
    if (RS) {
        sprintf( data + strlen(data), " got:");
        if (Cac1cmp) sprintf( data + strlen(data), " C1");
        if (Cac1fan) sprintf( data + strlen(data), " F1");
        if (Cac1fv) sprintf( data + strlen(data), " V1");
        if (Cac2cmp) sprintf( data + strlen(data), " C2");
        if (Cac2fan) sprintf( data + strlen(data), " F2");
        if (Cac2fv) sprintf( data + strlen(data), " V2");
    }
    if (diff) {
        sprintf( data + strlen(data), " MISSING:");
        if (diff&1) sprintf( data + strlen(data), " C1");
        if (diff&2) sprintf( data + strlen(data), " F1");
        if (diff&4) sprintf( data + strlen(data), " V1");
        if (diff&8) sprintf( data + strlen(data), " C2");
        if (diff&16) sprintf( data + strlen(data), " F2");
        if (diff&32) sprintf( data + strlen(data), " V2");
    }
    else sprintf( data + strlen(data), "    OK!  ");
    sprintf( data + strlen(data), " COMMS:%d sendBits:%d", COMMS, sendBits);
    log_message(DATA_FILE, data);
    
    /* for the first 8 cycles = 40 seconds - do not create or update the files that go out to
       other systems - sometimes there is garbage, which would be nice if is not sent at all */
    if ( ProgramRunCycles < 8 ) return;

    sprintf( data, ",AC1COMP,%5.3f\n_,AC1CND,%5.3f\n_,HE1I,%5.3f\n_,HE1O,%5.3f\n"\
    "_,AC2COMP,%5.3f\n_,AC2CND,%5.3f\n_,HE2I,%5.3f\n_,HE2O,%5.3f\n"\
    "_,WaterIN,%5.3f\n_,WaterOUT,%5.3f\n_,Tenv,%5.3f\n"\
    "_,Comp1,%d\n_,Fan1,%d\n_,Valve1,%d\n_,Comp2,%d\n_,Fan2,%d\n_,Valve2,%d",\
    Tac1cmp, Tac1cnd, The1i, The1o, Tac2cmp, Tac2cnd, The2i, The2o, Twi, Two, Tenv,\
    Cac1cmp, Cac1fan, Cac1fv, Cac2cmp, Cac2fan, Cac2fv);
    log_msg_ovr(TABLE_FILE, data);

    sprintf( data, "{AC1COMP:%5.3f,AC1CND:%5.3f,HE1I:%5.3f,HE1O:%5.3f,"\
    "AC2COMP:%5.3f,AC2CND:%5.3f,HE2I:%5.3f,HE2O:%5.3f,"\
    "WaterIN:%5.3f,WaterOUT:%5.3f,Tenv:%5.3f,"\
    "Comp1:%d,Fan1:%d,Valve1:%d,Comp2:%d,Fan2:%d,Valve2:%d}",\
    Tac1cmp, Tac1cnd, The1i, The1o, Tac2cmp, Tac2cnd, The2i, The2o, Twi, Two, Tenv,\
    Cac1cmp, Cac1fan, Cac1fv, Cac2cmp, Cac2fan, Cac2fv);
    log_msg_cln(JSON_FILE, data);
}

/* Function to get current time and put the hour in current_timer_hour */
void
GetCurrentTime() {
    static char buff[80];
    time_t t;
    struct tm *t_struct;
	
    t = time(NULL);
    t_struct = localtime( &t );
    strftime( buff, sizeof buff, "%H", t_struct );

    current_timer_hour = atoi( buff );
    
    strftime( buff, sizeof buff, "%m", t_struct );
    current_month = atoi( buff );
}

/* Turn ON compressor limitations:
    1 - it must be off
    2 - it must have been off for 8 minutes = 96 5 sec cycles
    3 - it must not be too hot 
    4 - the other compressor must not have been switched ON
         in the last 30 seconds 
    5 - for DEFROST - allow quick toggling */
unsigned short CanTurnC1On() {
    if (!cfg.use_ac1 || (Tac1cmp>59)) return 0;
    if (!Cac1cmp && (Cac1mode==4)) return 1;
    if (!Cac1cmp && (SCac1cmp > 96) &&
        ((Cac2cmp && (SCac2cmp > 6))||(!Cac2cmp))) return 1;
    else return 0;
}

/* Turn OFF compressor 1 limitations:
    1 - it must be ON
    2 - it must have been ON for at least 7 minutes = 84 5 sec cycles
    3 - during DEFROST cycle or power failure - can be turned off quicker */
unsigned short CanTurnC1Off() {
    if (Cac1cmp && ((Cac1mode>=4)||(COMMS==3))) return 1;
    if (Cac1cmp && (SCac1cmp > 84)) return 1;
    else return 0;
}

/* Turn ON/OFF fan limitations - fans can be toggled at will */
unsigned short CanTurnF1On() {
    return 1;
}

/* Turn ON/OFF fan limitations - fans can be toggled at will */
unsigned short CanTurnF1Off() {
    return 1;
}

/* Turn ON/OFF valve limitations:
    1 - to change a valve state - the compressor must be OFF
    2 - the compressor must have been OFF for 10 seconds = 2 cycles */
unsigned short CanTurnV1On() {
    if (!Cac1cmp && (SCac1cmp > 1)) return 1;
    else return 0;
}

/* Limitations are the same between valve on and valve off */
unsigned short CanTurnV1Off() {
    return CanTurnV1On();
}

/* Turn ON compressor limitations:
    1 - it must be off
    2 - it must have been off for 8 minutes = 96 5 sec cycles
    3 - it must not be too hot 
    4 - the other compressor must not have been switched ON
         in the last 30 seconds 
    5 - for DEFROST - allow quick toggling */
unsigned short CanTurnC2On() {
    if (!cfg.use_ac2 || (Tac2cmp>59)) return 0;
    if (!Cac2cmp && (Cac2mode==4)) return 1;
    if (!Cac2cmp && (SCac2cmp > 96) &&
        ((Cac1cmp && (SCac1cmp > 6))||(!Cac1cmp))) return 1;
    else return 0;
}

/* Turn OFF compressor 2 limitations:
    1 - it must be ON
    2 - it must have been ON for at least 7 minutes = 84 5 sec cycles
    3 - during DEFROST cycle or power failure - can be turned off quicker */
unsigned short CanTurnC2Off() {
    if (Cac2cmp && ((Cac2mode>=4)||(COMMS==3))) return 1;
    if (Cac2cmp && (SCac2cmp > 84)) return 1;
    else return 0;
}

/* Turn ON/OFF fan limitations - fans can be toggled at will */
unsigned short CanTurnF2On() {
    return 1;
}

/* Turn ON/OFF fan limitations - fans can be toggled at will */
unsigned short CanTurnF2Off() {
    return 1;
}

/* Turn ON/OFF valve limitations:
    1 - to change a valve state - the compressor must be OFF
    2 - the compressor must have been OFF for 10 seconds = 2 cycles */
unsigned short CanTurnV2On() {
    if (!Cac2cmp && (SCac2cmp > 1)) return 1;
    else return 0;
}

/* Limitations are the same between valve on and valve off */
unsigned short CanTurnV2Off() {
    return CanTurnV2On();
}

void TurnC1Off() { Cac1cmp = 0; SCac1cmp = 0;  }
void TurnC1On() { Cac1cmp = 1; SCac1cmp = 0; }
void TurnF1Off() { Cac1fan  = 0; SCac1fan = 0; }
void TurnF1On() { Cac1fan  = 1; SCac1fan = 0; }
void TurnV1Off() { Cac1fv  = 0; SCac1fv = 0; }
void TurnV1On() { Cac1fv  = 1; SCac1fv = 0; }
void TurnC2Off() { Cac2cmp = 0; SCac2cmp = 0; }
void TurnC2On() { Cac2cmp = 1; SCac2cmp = 0; }
void TurnF2Off() { Cac2fan  = 0; SCac2fan = 0; }
void TurnF2On() { Cac2fan  = 1; SCac2fan = 0; }
void TurnV2Off() { Cac2fv  = 0; SCac2fv = 0; }
void TurnV2On() { Cac2fv  = 1; SCac2fv = 0; }

short
SelectOpMode() {
    short StateDesired = 0;
    short wantC1on = 0;
    short wantF1on = 0;
    short wantV1on = 1;
    short wantC2on = 0;
    short wantF2on = 0;
    short wantV2on = 1;
    short nrACs_running = 0;
 //   static char data[280];
    short t = 0;
    
    if (Cac1cmp) nrACs_running++;
    if (Cac2cmp) nrACs_running++;
    if (HPL) { /* need to turn one AC on if possible */
        switch (nrACs_running) { /* check if we already have ACs running */
             case 0: /* both ACs are off - we need to decide which one we would want ON */
                t = cfg.use_ac1 + cfg.use_ac2;
                if (t==2) { /* both ACs are allowed - choose the one that has worked less */
                    if (C1RunCs <= C2RunCs) t=1;
                    else t=2;
                    /* if we selected the AC that is resting, and the other one can start - switch them */
                    if (t==1 && !CanTurnC1On() && CanTurnC2On()) t=2;
                    if (t==2 && !CanTurnC2On() && CanTurnC1On()) t=1;
                }
                switch (t) {
                    default:
                    case 0: /* no ACs allowed? */
                        break;
                    case 1: /* AC1 only */
                        wantC1on = 1;
                        break;
                    case 2: /* AC2 only */
                        wantC2on = 1;
                        break;
                }
                break;
             case 1: /* exactly 1 AC is already running - find out which one it is, and keep it running */
                if (Cac1cmp) { /* its AC1 */
                    wantC1on = 1;
                }
                else { /* its AC2 */
                    wantC2on = 1;
                }
                break;
             case 2: /* 2 ACs are running - we need to decide which one we want turned off - in practice we leave ON the other one*/
                    /* keep it simple - try to turn OFF the AC that has worked more in the long run */
                    if (C1RunCs>=C2RunCs) {
                        wantC2on = 1;
                    }
                    else {
                        wantC1on = 1;
                    }
                break;
        }
    }
    if (HPH) { /* need to turn both ACs if possible */
        wantC1on = 1;
        wantC2on = 1;
    }
    
    /* DEFROST mode is kinda special - make it work */
    if (Cac1mode==4) { wantC1on = 1; }
    if (Cac2mode==4) { wantC2on = 1; }
    
    /* get back out of OVH protection: compressor is OFF, mode is OHP, stayed like so for 2 mins */
    if (!Cac1cmp && (Cac1mode==5) && (SCac1mode>24)) { Cac1mode = 0; SCac1mode = 0; }
    if (!Cac2cmp && (Cac2mode==5) && (SCac2mode>24)) { Cac2mode = 0; SCac2mode = 0; }
    
    if (COMMS==3) { /* hwwm is signaling power has switched to battery */
        /* assume everything is OFF except the fourway valves */
        wantC1on = 0;
        wantF1on = 0;
        wantC2on = 0;
        wantF2on = 0;
    }

    /* Until now we alredy know which ACs we want on; check wantC_on for that.
        Now, it is time to manage running ACs mode, so that we keep the compressors within
        allowed working parameters */
    if (wantC1on) { 
        switch (Cac1mode) {
            case 0: /* AC 1 is in OFF mode: */
                    /* if AC1 can be turned ON and valve is ON, switch its mode to STARTING */
                    if (CanTurnC1On() && Cac1fv) {
                        Cac1mode = 1;
                        SCac1mode = 0;
                    }
                break;
            case 1: /* AC 1 is in STARTING mode: */
                    wantF1on = 1;
                    /* when the compressor temp reaches 56 - switch mode to COMP COOLING */
                    if (Tac1cmp>56) { 
                        Cac1mode = 2;
                        SCac1mode = 0;
                    }
                    /* if 2 minutes into starting - make mode FIN STACK HEATING */
                    if (SCac1mode>24) {
                        Cac1mode = 3;
                    }
                break;
            case 2: /* AC 1 is in COMP COOLING mode: */
                    /* when the compressor temp falls below 56 - switch mode to FIN STACK HEATING */
                    if ((Tac1cmp<56) && (SCac1mode>10)) {
                        Cac1mode = 3;
                        SCac1mode = 0;
                    }
                break;
            case 3: /* AC 1 is in FIN STACK HEATING mode: */
                    wantF1on = 1;
                    /* when the compressor temp goes back up to 56
                        switch mode to COMP COOLING */
                    if ((Tac1cmp>56) && (SCac1mode>10)) {
                        Cac1mode = 2;
                        SCac1mode = 0;
                    }
                    /* if enough time passes, fins stack gets seriosly cold - switch to defrost mode */
                    if ((SCac1mode>159) && (Tac1cnd<-6)) {
                        Cac1mode = 4;
                        SCac1mode = 0;
                    }
                    /* if FIN HEATING mode goes for 30 minutes and fins stay below -3 : DEFROST */
                    if ((SCac1mode>359) && (Tac1cnd<-3)) {
                        Cac1mode = 4;
                        SCac1mode = 0;
                    }
                break;
            case 4: /* AC1 is in DEFROST mode */
                    switch (SCac1mode) {
                        case 0 ... 8: /* ONLY VALVE ON */
                            wantV1on = 1;
                            wantC1on = 0;
                            wantF1on = 0;
                            break;
                        case 9 ... 17: /* ALL OFF - this switches mode to cooling, so fins become heating */
                            wantV1on = 0;
                            wantC1on = 0;
                            wantF1on = 0;
                            break;
                        case 18 ... 57: /* COMPRESSOR ON WITH VALVE OFF */
                            wantV1on = 0;
                            wantC1on = 1;
                            wantF1on = 0;
                            break;
                        case 58 ... 69: /* ALL OFF; prep to switch back to heating */
                            wantV1on = 0;
                            wantC1on = 0;
                            wantF1on = 0;
                            break;
                        case 70 ... 81: /* VALVE back ON */
                            wantV1on = 1;
                            wantC1on = 0;
                            wantF1on = 0;
                            break;
                    }
                    /* when the DEFROST is complete - switch mode back to STARTING*/
                    if (SCac1mode>=82) {
                        Cac1mode = 1;
                        SCac1mode = 0;
                    }
                break;
        }
    } else {
        /* only do the mode clean up if turning off the compressor is possible */
        if (Cac1mode && CanTurnC1Off()) {
            Cac1mode = 0;
            SCac1mode = 0;
        }
    }
    if (wantC2on) { 
        switch (Cac2mode) {
            case 0: /* AC 2 is in OFF mode, but we want it ON: */
                    /* if AC2 can be turned ON and valve is ON, switch its mode to STARTING */
                    if (CanTurnC2On() && Cac2fv) {
                        Cac2mode = 1;
                        SCac2mode = 0;
                    }
                break;
            case 1: /* AC 2 is in STARTING mode: */
                    wantF2on = 1;
                    /* when the compressor temp reaches 56 - switch mode to COMP COOLING */
                    if (Tac2cmp>56) { 
                        Cac2mode = 2;
                        SCac2mode = 0;
                    }
                    /* if 2 minutes into starting - make mode FIN STACK HEATING */
                    if (SCac2mode>24) {
                        Cac2mode = 3;
                    }
                break;
            case 2: /* AC 2 is in COMP COOLING mode: */
                    /* when the compressor temp falls below 56 - switch mode to FIN STACK HEATING */
                    if ((Tac2cmp<56) && (SCac2mode>10)) {
                        Cac2mode = 3;
                        SCac2mode = 0;
                    }
                break;
            case 3: /* AC 2 is in FIN STACK HEATING mode: */
                    wantF2on = 1;
                    /* when the compressor temp goes back up to 56
                        switch mode to COMP COOLING */
                    if ((Tac2cmp>56) && (SCac2mode>10)) {
                        Cac2mode = 2;
                        SCac2mode = 0;
                    }
                   /* if enough time passes, fins stack gets seriosly cold - switch to defrost mode */
                    if ((SCac2mode>159) && (Tac2cnd<-6)) {
                        Cac2mode = 4;
                        SCac2mode = 0;
                    }
                    /* if FIN HEATING mode goes for 30 minutes and fins stay below -3 : DEFROST */
                    if ((SCac2mode>359) && (Tac2cnd<-3)) {
                        Cac2mode = 4;
                        SCac2mode = 0;
                    }
                break;
            case 4: /* AC2 is in DEFROST mode */
                    switch (SCac2mode) {
                        case 0 ... 8: /* ONLY VALVE ON */
                            wantV2on = 1;
                            wantC2on = 0;
                            wantF2on = 0;
                            break;
                        case 9 ... 17: /* ALL OFF - this switches mode to cooling, so fins become heating */
                            wantV2on = 0;
                            wantC2on = 0;
                            wantF2on = 0;
                            break;
                        case 18 ... 57: /* COMPRESSOR ON WITH VALVE OFF */
                            wantV2on = 0;
                            wantC2on = 1;
                            wantF2on = 0;
                            break;
                        case 58 ... 69: /* ALL OFF; prep to switch back to heating */
                            wantV2on = 0;
                            wantC2on = 0;
                            wantF2on = 0;
                            break;
                        case 70 ... 81: /* VALVE back ON */
                            wantV2on = 1;
                            wantC2on = 0;
                            wantF2on = 0;
                            break;
                    }
                    /* when the DEFROST is complete - switch mode back to STARTING*/
                    if (SCac2mode>=82) {
                        Cac2mode = 1;
                        SCac2mode = 0;
                    }
                break;
        }
    } else {
        /* only do the mode clean up if turning off the compressor is possible */
        if (Cac2mode && CanTurnC2Off()) {
            Cac2mode = 0;
            SCac2mode = 0;
        }
    }

    /* as a final action - if an AC is not allowed to be used by config file - make sure it stays OFF */
    if (!cfg.use_ac1) {
        wantC1on = 0;
        wantF1on = 0;
        wantV1on = 0;
    }
    if (!cfg.use_ac2) {
        wantC2on = 0;
        wantF2on = 0;
        wantV2on = 0;
    }

    /* Turning to OFF mode cleanup:
        If an AC has been ON, but now will be turned OFF - change its mode accordingly, if possible */
    if (Cac1mode!=4 && CanTurnC1Off()) {
        if (Cac1cmp && !wantC1on) {
            Cac1mode = 0;
            SCac1mode = 0;
        }
    }
    if (Cac2mode!=4 && CanTurnC2Off()) {
        if (Cac2cmp && !wantC2on) {
            Cac2mode = 0;
            SCac2mode = 0;
        }
    }

    /* compressors overheating protection: if running and hotter than 63 C */
    if (Cac1cmp && (Tac1cmp>63)) {
        /* switch mode to OHP, turn compressor and fan OFF */
        Cac1mode = 5;
        SCac1mode = 0;
        wantC1on = 0;
        wantF1on = 0;
    }
    if (Cac2cmp && (Tac2cmp>63)) {
        /* switch mode to OHP, turn compressor and fan OFF */
        Cac2mode = 5;
        SCac2mode = 0;
        wantC2on = 0;
        wantF2on = 0;
    }

    if ( wantC1on) StateDesired |= 1;
    if ( wantF1on ) StateDesired |= 2;
    if ( wantV1on )  StateDesired |= 4;
    if ( wantC2on )  StateDesired |= 8;
    if ( wantF2on ) StateDesired |= 16;
    if ( wantV2on )  StateDesired |= 32;
    return StateDesired;
}

void
ActivateDevicesState(const unsigned short _ST_) {
    unsigned short current_state = 0;
    unsigned short new_state = 0;

    /* calculate current state */
    if ( Cac1cmp ) current_state |= 1;
    if ( Cac1fan ) current_state |= 2;
    if ( Cac1fv ) current_state |= 4;
    if ( Cac2cmp ) current_state |= 8;
    if ( Cac2fan ) current_state |= 32;
    if ( Cac2fv ) current_state |= 64;
    /* make changes as needed */
    /* _ST_'s bits describe the peripherals desired state:
        bit 1  (1) - compressor 1
        bit 2  (2) - fan 1
        bit 3  (4) - fourway valve 1
        bit 4  (8) - compressor 2
        bit 5 (16) - fan 2
        bit 6 (32) - fourway valve 2 */
    if (_ST_ &   1)  { if (CanTurnC1On()) TurnC1On(); } else { if (CanTurnC1Off()) TurnC1Off(); }
    if (_ST_ &   2)  { if (CanTurnF1On()) TurnF1On(); } else { if (CanTurnF1Off()) TurnF1Off(); }
    if (_ST_ &   4)  { if (CanTurnV1On()) TurnV1On(); } else { if (CanTurnV1Off()) TurnV1Off(); }
    if (_ST_ &   8)  { if (CanTurnC2On()) TurnC2On(); } else { if (CanTurnC2Off()) TurnC2Off(); }
    if (_ST_ &  16) { if (CanTurnF2On()) TurnF2On(); } else { if (CanTurnF2Off()) TurnF2Off(); }
    if (_ST_ &  32) { if (CanTurnV2On()) TurnV2On(); } else { if (CanTurnV2Off()) TurnV2Off(); }
    
    SCac1cmp++;
    SCac1fan++;
    SCac1fv++;
    SCac1mode++;
    SCac2cmp++;
    SCac2fan++;
    SCac2fv++;
    SCac2mode++;

    /* calculate desired new state */
    if ( Cac1cmp ) { new_state |= 1; C1RunCs++; }
    if ( Cac1fan ) new_state |= 2;
    if ( Cac1fv ) new_state |= 4;
    if ( Cac2cmp ) { new_state |= 8; C2RunCs++; }
    if ( Cac2fan ) new_state |= 32;
    if ( Cac2fv ) new_state |= 64;
    /* if current state and new state are different... */
    if ( current_state != new_state ) {
        /* then put state on GPIO pins - this prevents lots of toggling at every 10s decision */
        ControlStateToGPIO();
    }
}


/* Gets called after main decision making for the cycle has taken place. This means that
   control states can be used to make decisions. The task here is to take into account limitations, 
   and compute an answer to return to hwwm */
void
ComputeSendBits() {
    unsigned short nrACs_startable = 0;
    unsigned short nrACs_stoppable = 0;
    unsigned short k = 0;

    /* Start by determining how many AC we can START */
    if (CanTurnC1On() && (Cac1mode!=4)) nrACs_startable++;
    if (CanTurnC2On() && (Cac2mode!=4)) nrACs_startable++;

    /* Then determine how many AC we can STOP */
    if (CanTurnC1Off() && (Cac1mode!=4)) nrACs_stoppable++;
    if (CanTurnC2Off() && (Cac2mode!=4)) nrACs_stoppable++;
    
    /* When cfg.mode is set to OFF -  allow all changes */
    if (!cfg.mode) {
        sendBits = 0;
        return;
    }

    /* If we cannot start or stop ACs - do not allow changes */
    if (!(nrACs_startable || nrACs_stoppable)) {
        sendBits = 0;
        return;
    }

    /* What follows here is the smallest possible code I came up with to give the desired 
       results as seen in a all-possible-scenarios-blown-up table; 
       Basically, this is magic! ;) */
    if (nrACs_startable) { k = 1; }
    if (nrACs_startable==nrACs_stoppable) { k = 0; }
    if (nrACs_stoppable) {
        k += 1 + nrACs_stoppable + nrACs_startable;
    }
    
    sendBits = k;
}

int
main(int argc, char *argv[])
{
    /* set iter to its max value - makes sure we get a clock reading upon start */
    unsigned short iter = 30;
    unsigned short iter_P = 0;
    unsigned short DevicesWantedState = 0;
    struct timeval tvalBefore, tvalAfter;

    SetDefaultCfg();

    /* before main work starts - try to open the log files to write a new line
    ...and SCREAM if there is trouble! */
    if (log_message(LOG_FILE,"***")) {
        printf("Cannot open the mandatory "LOG_FILE" file needed for operation!\n");
        exit(3);
    }
    if (log_message(DATA_FILE,"***")) {
        printf("Cannot open the mandatory "DATA_FILE" file needed for operation!\n");
        exit(4);
    }
    if (log_message(TABLE_FILE,"***")) {
        printf("Cannot open the mandatory "TABLE_FILE" file needed for operation!\n");
        exit(5);
    }
    if (log_message(JSON_FILE,"***")) {
        printf("Cannot open the mandatory "JSON_FILE" file needed for operation!\n");
        exit(6);
    }
    if (log_message(CFG_TABLE_FILE,"***")) {
        printf("Cannot open the mandatory "CFG_TABLE_FILE" file needed for operation!\n");
        exit(7);
    }

    daemonize();

    write_log_start();

    just_started = 3;

    parse_config();

    ReadPersistentData();

    /* Enable GPIO pins */
    if ( ! EnableGPIOpins() ) {
        log_message(LOG_FILE,"ALARM: Cannot enable GPIO! Aborting run.");
        exit(11);
    }

    /* Set GPIO directions */
    if ( ! SetGPIODirection() ) {
        log_message(LOG_FILE,"ALARM: Cannot set GPIO direction! Aborting run.");
        exit(12);
    }

    /* By default all control states are 0 == OFF;
    With putting output pins to OFF, we make sure that relay will obey
    inverting output setting of config file at startup, and thus avoid
    an unnecessary very short toggling of output relays on startup */
    ControlStateToGPIO();

    do {
        /* Do all the important stuff... */
        if ( gettimeofday( &tvalBefore, NULL ) ) {
            log_message(LOG_FILE,"WARNING: error getting tvalBefore...");
        }
        /* get the current hour every 5 minutes */
        if ( iter == 60 ) {
            iter = 0;
            GetCurrentTime();
            /* and increase counter controlling writing out persistent power use data */
            iter_P++;
            if ( iter_P == 2) {
                iter_P = 0;
                WritePersistentData();
            }
        }
        iter++;
        ReadSensors();
        ReadCommsPins();
        /* Apply sensors data corrections */
        Twi += cfg.wicorr;
        Two += cfg.wocorr;
        Tenv += cfg.tenvcorr;
        /* if MODE is not 0==OFF, work away */
        if (cfg.mode) {
            /* process sensors data here, and decide what devices should do */
            DevicesWantedState = SelectOpMode();
        } else {
            DevicesWantedState = 0;
        }
        ActivateDevicesState(DevicesWantedState);
        ComputeSendBits();
        WriteCommsPins();
        LogData(DevicesWantedState);
        ProgramRunCycles++;
        if ( just_started ) { just_started--; }
        if ( need_to_read_cfg ) {
            need_to_read_cfg = 0;
            just_started = 1;
            parse_config();
        }
        if ( gettimeofday( &tvalAfter, NULL ) ) {
            log_message(LOG_FILE,"WARNING: error getting tvalAfter...");
            sleep( 1 );
        }
        else {
            /* use hardcoded sleep() if time is skewed (for eg. daylight saving, ntp adjustments, etc.) */
            if ((tvalAfter.tv_sec - tvalBefore.tv_sec) > 5) {
                sleep( 1 );
            }
            else {
                /* otherwise we have valid time data - so calculate exact sleep time
                so period between active operations is bang on 5 seconds */
                usleep(5000000 - (((tvalAfter.tv_sec - tvalBefore.tv_sec)*1000000L \
                + tvalAfter.tv_usec) - tvalBefore.tv_usec));
            }
        }
    } while (1);

    /* Disable GPIO pins */
    if ( ! DisableGPIOpins() ) {
        log_message(LOG_FILE,"ALARM: Cannot disable GPIO on UNREACHABLE exit!");
        return(222);
    }

    log_message(LOG_FILE,"ALARM: Something wierd happened. Reached an UNREACHABLE part of the program!");

    return(225);
}

/* EOF */
