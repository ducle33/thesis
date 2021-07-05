
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
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <ros/time.h>

//#define DEBUG/
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

unsigned long last_ms;
unsigned long ms;

// WIFI CONFIGS ===============================


const char* ssid     = "Phuong";
const char* password = "0985587344";
// Set the rosserial socket server IP address
IPAddress server(192,168,1,7);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;
void cmd_velCallback(const geometry_msgs::Twist& CVel);

// ROS CONFIGS ==============================

ros::NodeHandle nh;

// Make a chatter publisher
ros::Subscriber<geometry_msgs::Twist> Sub("/cmd_vel", &cmd_velCallback );

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

// OTHER CONFIGS ==========================
double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";


// Print byffer
void print_data()
{
  uint8_t i;

  #ifndef DEBUG
    Serial.write(0x16);
  Serial.write(0x16);
  Serial.write(0x16);
  for (i =0 ; i<sizeof(data); i++)
  {
    Serial.write(data[i]);
  }
  #else
  Serial.print(0x16);
  Serial.print(0x16);
  Serial.print(0x16);
  for (i =0 ; i<sizeof(data); i++)
  {
    Serial.print(data[i]);
  }
  Serial.println();
  #endif 
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
    }
}

// Calc checksum
uint32_t checksum(byte *d, uint8_t from, uint8_t to)
{
    uint32_t _crc = 0;
    uint8_t i = 0;
    for (i = from; i < to; i++)
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

    uint32_t crc = checksum(data, 2, 10);
    
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

void resolveRxFrame(byte * d, double * x, double * y, double * th)
{
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;
    uint32_t tmp3 = 0;
    uint32_t rx_crc, local_crc;
    
    rx_crc = d[26] << 24 | d[27]<<16 | d[28]<<8 | d[29]; 
    
    tmp1 = d[2] << 56 | d[3]<< 48 | d[4]<< 40 | d[5] << 32 | d[6] << 24 | d[7] << 16 | d[8] << 8 | d[9];
    tmp2 = d[10] << 56 | d[11]<< 48 | d[12]<< 40 | d[13] << 32 | d[14] << 24 | d[15] << 16 | d[16] << 8 | d[17];
    tmp3 = d[18] << 56 | d[19]<< 48 | d[20]<< 40 | d[21] << 32 | d[22] << 24 | d[23] << 16 | d[24] << 8 | d[25];;
    
    local_crc = checksum(d, 2, 26);

    if (local_crc == rx_crc)
    {
        *x = *(double *)&tmp1;
        *y = *(double *)&tmp2;
        *th = *(double *)&tmp3;
    }
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
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

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
  pinMode(2, OUTPUT); 
  digitalWrite(2, 1);
  setup_wifi();
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(Sub);
  last_f_linear = 0.0f;
  last_f_angular = 0.0f;
  digitalWrite(2, 0);
  broadcaster.init(nh);
}

char c, last_c;
bool frame_get_ready = false;
byte rx_frame[30];
int i  = 0;
void loop()
{
   double x,y,th;
   while(Serial.available())
  {
    c = Serial.read();
    rx_frame[i] = c;
    i++;
     if (c == '\n' && last_c == '\r')
     {
        resolveRxFrame((byte *)rx_frame, &x, &y, &th);
//        Serial.print("RX -> ");
//        Serial.println(rx_frame);
        Serial.print("Rx -> X= ");
        Serial.print(x, 5);
        Serial.print(", Y=");
        Serial.print(y, 5);
        Serial.print(", THETA=");
        Serial.println(th, 5);
        i = 0;
     }
     last_c = c;
  }
  // When ros connected 
  if (nh.connected()) {
    
      // Send cmd_vel data to base control
      parseTxFrame(data, f_linear, f_angular);
      print_data(); 
      
      // drive in a circle
      double dx = 0.2;
      double dtheta = 0.18;
      x += cos(theta)*dx*0.1;
      y += sin(theta)*dx*0.1;
      theta += dtheta*0.1;
      if(theta > 3.14)
        theta=-3.14;
        
      // tf odom->base_link
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      
      t.transform.rotation = tf::createQuaternionFromYaw(theta);
      t.header.stamp = nh.now();
      
      broadcaster.sendTransform(t);

      // Blink status led
      if (ms>= 50)
      {
         ms = 0;
         digitalWrite(2, 0);
      }      
      else 
      {
        digitalWrite(2, 1);
      }
  }
  else 
  {
      if (ms>= 10)
      {
         ms = 0;
         state = !state;
         digitalWrite(2, state);
      }
  }
  ms++;
  delay(100);
  last_f_linear = f_linear;
  last_f_angular = f_angular;
  nh.spinOnce();
}
