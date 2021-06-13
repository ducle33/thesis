
/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * This intend to connect to a Wifi Access Point
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */
#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

// FRAME CONFIGS ===============================
byte data[16];

byte b_linear[4];

byte state = 0;
uint32_t checksum(byte *d);
uint8_t parseChecksum(byte *d, uint32_t crc);
void parseTxFrame(byte *d /* uint8_t *d */, float vel_linear, float vel_angular);
void float2byte(float *f, byte *d, byte s);

float f_linear;
float f_angular;


float last_f_linear;
float last_f_angular;

// WIFI CONFIGS ===============================


const char* ssid     = "Phuong";
const char* password = "0985587344";
// Set the rosserial socket server IP address
IPAddress server(192,168,1,6);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;
void cmd_velCallback(const geometry_msgs::Twist& CVel);

// ROS CONFIGS ==============================

ros::NodeHandle nh;

// Make a chatter publisher
std_msgs::String str_msg;

ros::Publisher chatter("chatter", &str_msg);

ros::Subscriber<geometry_msgs::Twist> Sub("/cmd_vel", &cmd_velCallback );

// OTHER CONFIGS ===========================
// Be polite and say hello
char hello[13] = "hello world!";

// Print byffer
void print_data()
{
  uint8_t i;
  state =!state;

  for (i =0 ; i<sizeof(data); i++)
  {
    #ifdef DEBUG
    Serial.print(data[i], HEX);
    Serial.print(" ");
    #else
    Serial.print(data[i]);
    #endif
  }
  Serial.println();

}

// Aux fucntion
void float2byte(float *f, byte *d, byte s)
{
    uint32_t * tmp;
    tmp = (uint32_t*)f;
    uint8_t i = 0;
    for (i = 0; i<s; i++)
    {
        
        d[i] = *tmp>>(24-8*i);
        // printf("byte[%d] 0x%x\n", i, d[i]);
    }
}

// Calc checksum
uint32_t checksum(byte *d)
{
    uint32_t _crc = 0;
    uint8_t i = 0;
    for (i = 2; i < 10; i++)
    {
        _crc+= d[i];
    }
    return _crc;
}

// Parse frames for transmition
void parseTxFrame(byte *d /* uint8_t *d */, float vel_linear, float vel_angular)
{
    // Header
    d[0] = 1;
    d[1] = 2; 
  
    // Resolve float to 4 bytes binary.
    float2byte(&vel_linear, &d[2], 4);
    float2byte(&vel_angular, &d[6], 4);

    uint32_t crc = checksum(data);
    
    #ifdef DEBUG
    Serial.print("CRC = ");
    Serial.println(crc);
    #endif
    
    d[10] = (crc >> 24) & 0xFF;
    d[11] = (crc >> 16)  & 0xFF; 
    d[12] = (crc >> 8)  & 0xFF; 
    d[13] = (crc )  & 0xFF; 
    
    // EOF
    d[14] = '\r';
    d[15] = '\n';
}




float * resolveRxFrame(byte * d)
{
    float vel[2] = {};
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;
    uint32_t rx_crc, local_crc;
    
    rx_crc = d[10] << 24 | d[11]<<16 | d[12]<<8 | d[13]; 
    tmp1 = d[2] << 24 | d[3]<<16 | d[4]<<8 | d[5];
    tmp2 = d[6] << 24 | d[7]<<16 | d[8]<<8 | d[9];
    
    local_crc = checksum(d);

    if (local_crc == rx_crc)
    {
        vel[0] = *(float *)&tmp1;
        vel[1] = *(float *)&tmp2;
    }
    
    #ifdef DEBUG
    Serial.print("Linear vel = ");
    Serial.println(vel[0]);
    Serial.print("Angular vel = ");
    Serial.println(vel[1]);
    #endif

    return vel;
}


// ROS Callback
void cmd_velCallback( const geometry_msgs::Twist& CVel){
    //geometry_msgs::Twist twist = twist_msg;   
    double d_linear  = CVel.linear.x;
    double d_angular = CVel.angular.z;

    f_linear = (float) d_linear;
    f_angular = (float) d_angular;

    #ifdef DEBUG
    Serial.print("cmd_vel: ");
    Serial.print(f_linear);
    Serial.print(" | ");
    Serial.println(f_angular);
    #endif
}


void setup_wifi()
{
    // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  pinMode(2, OUTPUT); 
//  Serial.println();
//  Serial.print("Connecting to ");
//  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
  }
//  Serial.println("");
//  Serial.println("WiFi connected");
//  Serial.println("IP address: ");
//  Serial.println(WiFi.localIP());

  // Broadcast ready state
  memset(data, 0, sizeof(data));
  data[0] = 0xF0;
  data[1] = 0xF0;
  data[2] = 0xF0;
  print_data(); 
  memset(data, 0, sizeof(data));
}

void setup()
{
  // Set up connection
  setup_wifi();
  
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Start to be polite
  nh.advertise(chatter);
  nh.subscribe(Sub);
  last_f_linear = 0.0f;
  last_f_angular = 0.0f;
}

void loop()
{

  if (nh.connected()) {
      // Say hello
      str_msg.data = hello;
      chatter.publish( &str_msg );
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  if (f_linear != last_f_linear || f_angular != last_f_angular )
  {
      parseTxFrame(data, f_linear, f_angular);
      print_data(); 
  }
  last_f_linear = f_linear;
  last_f_angular = f_angular;
  digitalWrite(2, state);
  delay(100);
}
