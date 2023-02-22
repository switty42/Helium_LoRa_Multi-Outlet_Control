/* 
   MultiOutlet Helium project  - 3 Outlet control
   Coder - Stephen Witty
 
   Pumpt project used sample code with needed changes from Rak GitHub site.  Code was copied from github on 5-20-22 10am
   The sample code was a LoraWAN communication example from Rak GitHub site.    

   Must copy Helium keys into keys.h to connect to Helium LoRa network

   Project should be considered demo quality only

   Known Bugs - code likely does not support millis() rollover properly, reset MCU before 49 days of operation

   Version history
   1 - 7-16-22 Copied Pump project code from SWitty repository on laptop.  Pump project was single outlet control
   2 - 7-20-22 Add Stress test mode
   3 - 2-22-23 Cleaned up to publish to GitHub
/*

/**
 * @file LoRaWAN_OTAA_ABP.ino
 * @author rakwireless.com
 * @brief LoRaWan node example with OTAA/ABP registration
 * @version 0.1
 * @date 2020-08-21
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */

#define VERSION "3"
 
#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include "keys.h"

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                      /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0                   /*LoRaMac datarates definition, from DR_0 to DR_5*/
//W @WW suggests this should be 3 for connectivity issues
#define LORAWAN_TX_POWER TX_POWER_5             /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3                      /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;         /* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;    /* Region:EU868*/ /*W change to US915 */
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;         /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                      /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};
//W @WW suggests this should be ADR_OFF for connectivity issues

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

/*Global variables ****************************************************************/
bool sent_message=false;
bool received_message=false;
bool helium_connected=false;
bool helium_connect_failed=false;
bool helium_send_failed=false;
bool payload_error=false;
bool stress_test_go=false;

int run_relay_time[3]={0,0,0};  //Hold time one of the three relays will run, zero if not active

/**************************************************************************************/

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);  //Helium send GREEN
  pinMode(WB_IO1,OUTPUT); //Relay1 operation
  pinMode(WB_IO3,OUTPUT); //Relay2 operation
  pinMode(WB_IO4,OUTPUT); //Relay3 operation
  pinMode(WB_IO5,OUTPUT); //Helium Receive BLUE
  pinMode(WB_IO6,OUTPUT); //Error (Helium connection or send error)RED
  
  //RAK5005-O LED1 pin is connected to and controlled by LED_BULTIN which is a Green LED
  //This project connects the LED1 pin on the RAK5005-O board to an external Green LED and resistor
  //This results in the Green external LED having the same light pattern as the LED_BULTIN light
  
  digitalWrite(LED_BUILTIN,LOW);
  digitalWrite(WB_IO1,LOW);
  digitalWrite(WB_IO3,LOW);
  digitalWrite(WB_IO4,LOW);
  digitalWrite(WB_IO5,LOW);
  digitalWrite(WB_IO6,LOW);

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }

  // Initialize LoRa chip.
  lora_rak4630_init();

  Serial.println("=====================================");
  Serial.print("MultiOutlet Application Version: ");
  Serial.println(VERSION);
  
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }

  switch (g_CurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
  case LORAMAC_REGION_CN779:
    Serial.println("Region: CN779");
    break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
    break;
  case LORAMAC_REGION_RU864:
    Serial.println("Region: RU864");
    break;
  case LORAMAC_REGION_AS923_2:
    Serial.println("Region: AS923-2");
    break;
  case LORAMAC_REGION_AS923_3:
    Serial.println("Region: AS923-3");
    break;
  case LORAMAC_REGION_AS923_4:
    Serial.println("Region: AS923-4");
      break;
  }
  Serial.println("=====================================");
  
  //creat a user timer to send data to server period
  uint32_t err_code;
  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
    return;
  }

  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }

  // Initialize LoRaWan
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();
}

//These are used in loop for timer counters etc
long sent_timer=0,failed_timer=0,receive_timer=0;
long run_relay_timer_end[3]={0,0,0};
String message_buffer="";

void loop()
{ 
  //Flash all LEDs when trying to connect to Helium at boot time, stay in loop until connect or fail
  while(helium_connected==false && helium_connect_failed==false)
  {
    digitalWrite(WB_IO6,HIGH);
    digitalWrite(WB_IO5,HIGH);
    digitalWrite(LED_BUILTIN,HIGH);
    delay(150);
    digitalWrite(WB_IO6,LOW);
    digitalWrite(WB_IO5,LOW);
    digitalWrite(LED_BUILTIN,LOW);
    delay(150);
  }
  
  if (helium_connect_failed==true) //No recovery from this condition, blink red error LED
  {
    Serial.println("ERROR - Connection to Helium failed, no retry!!!!!!!!!");
    while(true)
    {
      delay(100);
      digitalWrite(WB_IO6,HIGH);
      delay(100);
      digitalWrite(WB_IO6,LOW);
    }
  }

  if (stress_test_go==true)
  {
    stress_test();
    stress_test_go=false;
  }

  if (sent_message==true)  //Helium message sent, blink green LED
  {
    if (sent_timer==0) { sent_timer=millis()+1500; digitalWrite(LED_BUILTIN,HIGH); }
    if (sent_timer<millis()) { sent_message=false; sent_timer=0; digitalWrite(LED_BUILTIN,LOW); }
  }

  if (helium_send_failed==true || payload_error==true)  //Helium message send fails or payload parse error, blink red LED
  {
    if (failed_timer==0) { failed_timer=millis()+1500; digitalWrite(WB_IO6,HIGH); }
    if (failed_timer<millis()) { helium_send_failed=false; payload_error=false; failed_timer=0; digitalWrite(WB_IO6,LOW); }
  }

  if (received_message==true)  //Helium message received, blink blue LED
  {
    if (receive_timer==0) 
    { 
      receive_timer=millis()+1500; 
      digitalWrite(WB_IO5,HIGH);

      if (parse_payload(message_buffer)==false) payload_error=true;     
    }
    
    if (receive_timer<millis()) { received_message=false; receive_timer=0; digitalWrite(WB_IO5,LOW); }
  }

/************************************************* Begin relay 1 */
  if (run_relay_time[0]>0)  //Relay 1 logic
  { 
    if (run_relay_timer_end[0]==0)
    {
      run_relay_timer_end[0]=millis()+run_relay_time[0]*1000;
      digitalWrite(WB_IO1,HIGH);
      Serial.println("Turn ON relay 1");
    }

    if (run_relay_timer_end[0]<millis()) 
    { 
      run_relay_time[0]=0;
      run_relay_timer_end[0]=0;
      digitalWrite(WB_IO1,LOW);
      Serial.println("Turn OFF relay 1");
    }
  }
/***************************** End relay 1 ***********/

/************************************************* Begin relay 2 */
  if (run_relay_time[1]>0)  //Relay 2 logic
  { 
    if (run_relay_timer_end[1]==0)
    {
      run_relay_timer_end[1]=millis()+run_relay_time[1]*1000;
      digitalWrite(WB_IO3,HIGH);
      Serial.println("Turn ON relay 2");
    }

    if (run_relay_timer_end[1]<millis()) 
    { 
      run_relay_time[1]=0; 
      run_relay_timer_end[1]=0;
      digitalWrite(WB_IO3,LOW);
      Serial.println("Turn OFF relay 2");
    }
  }
/***************************** End relay 2 ***********/

/************************************************* Begin relay 3 */
  if (run_relay_time[2]>0)  //Relay 3 logic
  { 
    if (run_relay_timer_end[2]==0)
    {
      run_relay_timer_end[2]=millis()+run_relay_time[2]*1000;
      digitalWrite(WB_IO4,HIGH);
      Serial.println("Turn ON relay 3");
    }

    if (run_relay_timer_end[2]<millis()) 
    { 
      run_relay_time[2]=0; 
      run_relay_timer_end[2]=0;
      digitalWrite(WB_IO4,LOW);
      Serial.println("Turn OFF relay 3");
    }
  }
/***************************** End relay 3 ***********/
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  if(doOTAA == true)
  {
  Serial.println("OTAA Mode, Network Joined!");
  }
  else
  {
    Serial.println("ABP Mode");
  }

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);

    helium_connected=true; //Set helium success flag
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");

  helium_connect_failed=true;  //Set helium failed flag
}
/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{ 
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
  
 // relay_run_time=atoi((char *) app_data->buffer);
  received_message=true; 
  message_buffer=(char *) app_data->buffer;
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = 'H';
  m_lora_app_data.buffer[i++] = 'e';
  m_lora_app_data.buffer[i++] = 'l';
  m_lora_app_data.buffer[i++] = 'l';
  m_lora_app_data.buffer[i++] = 'o';
  m_lora_app_data.buffer[i++] = '!';
  m_lora_app_data.buffsize = i;

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);

    sent_message=true; //Sent message
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
    helium_send_failed=true;
  }
}

/**@brief Function for handling user timerout event.
 */
void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
  Serial.println("Sending frame now...");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}

bool parse_payload(String storage)
{
  int a, count=0, parms=0;
  int decode[3][2];
  char buffer[100];

  buffer[0]=NULL;
  
  Serial.print("(");
  Serial.print(storage);
  Serial.println(")");

  if (storage.length()>50)
  {
    Serial.println("ERROR - Length of payload String is too large");
    return false;
  }

  //If receive stress test command, break out early, return and run
  if (storage=="stress test") { stress_test_go=true; return true; }

  storage.toCharArray(buffer,50);
  
  if (strlen(buffer)<3 || strlen(buffer)>17)
    {
      Serial.println("ERROR - Length of payload buffer array is invalid");
      return false;
    }
  for(a=0;a<strlen(buffer);a++) if (isDigit(buffer[a])==false && buffer[a]!=' ') break;
  if (a!=strlen(buffer))
  {
    Serial.println(a);
    Serial.println("ERROR - payload is not all digits");
    return false; 
  }
  if (buffer[0]==' ' || buffer[strlen(buffer)-1]==' ')
  {
    Serial.println("Error - Payload starts or stops with a space");
    return false;
  }
  for(a=0;a<buffer[strlen(buffer)-2];a++)
    if(buffer[a]==' ' && buffer[a+1]==' ') break;
  if (a!=buffer[strlen(buffer)-2])
  {
    Serial.println("Error - payload has two spaces in a row");
    return false;
  }

  for(a=0;a<strlen(buffer);a++)
    if (buffer[a]==' ') count++;
  if (count !=1 && count !=3 && count !=5)
  {
    Serial.println("Error - parm number not correct");
    return false;
  }

  for(a=0;a<3;a++)  //Setting all vars to something valid just for safety
  {
    decode[a][0]=0;
    decode[a][1]=0;
  }

  //Over writing preset numbers with whatever is in the buffer string, it can have 1 to 3 sets of numbers
  parms=sscanf(buffer,"%d %d %d %d %d %d",&decode[0][0],&decode[0][1],&decode[1][0],&decode[1][1],&decode[2][0],&decode[2][1]);

  if (parms !=2 && parms !=4 && parms !=6)
  {
    Serial.println("Error - parm decode problem in sscanf logic");
    return false;
  }

  for(a=0;a<parms/2;a++) if(decode[a][0]<1 || decode[a][0]>3 || decode[a][1]<5 || decode[a][1]>300) break;
  if (a!=parms/2)
  {
    Serial.println(parms);
    Serial.println(a);
    Serial.println("Error - parm not in range");
    return false;
  }

  //run_relay_time array starts at zero, but physical relays are numbered from 1 so -1 below
  for(a=0;a<parms/2;a++) run_relay_time[decode[a][0]-1]=decode[a][1];

  return true;
}

void stress_test(void)
{
 int a,b=50;
 Serial.println("Start Stress Test!!!!!!!!!!!!!!!!!!");
 for (a=800;a>10;a=a-b)
 {
      Serial.print("Time Value: ");
      Serial.println(a);
      digitalWrite(WB_IO1,HIGH);
      delay(a);
      digitalWrite(WB_IO1,LOW);

      digitalWrite(WB_IO3,HIGH);
      delay(a);
      digitalWrite(WB_IO3,LOW);

      digitalWrite(WB_IO4,HIGH);
      delay(a);
      digitalWrite(WB_IO4,LOW);

      if (a==100) b=1;
 }
 for (a=0;a<50;a++)
 {
  Serial.print("Time Value: ");
  Serial.println(a);
  digitalWrite(WB_IO1,HIGH);
  delay(10);
  digitalWrite(WB_IO1,LOW);

  digitalWrite(WB_IO3,HIGH);
  delay(10);
  digitalWrite(WB_IO3,LOW);

  digitalWrite(WB_IO4,HIGH);
  delay(10);
  digitalWrite(WB_IO4,LOW);
 }

 Serial.println("Stop Stress Test........");
}
