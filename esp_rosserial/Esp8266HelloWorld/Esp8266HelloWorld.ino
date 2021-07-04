
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

void resolveRxFrame(const uint8_t * d, double * x, double * y, double * th)
{
    uint32_t rx_crc = d[26] << 24 | d[27]<<16 | d[28]<<8 | d[29]; 
    uint32_t local_crc = checksum(d, 2, 26);
    if (local_crc == rx_crc)
    {
        double tmp[3];
        uint8_t i;
        for (i=0; i<3; i++)
        {
            union {
              double r;
              uint8_t rbytes[8];
            } rcv_double;
            uint8_t ii;
            for (ii=0; ii<8; ii++) 
            {
                rcv_double.rbytes[7-ii] = d[8*i+ii+2];
            }
            tmp[i] = rcv_double.r;
        }
        *x = tmp[0];
        *y = tmp[1];
        *th = tmp[2];
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
unsigned char rx_frame[32];
int i  = 0;
void loop()
{
  double dx,dy,dth;
   while(Serial.available())
  {
    c = Serial.read();
    if (i == 0 && c == 2)
    {
        i++;      
    }
    rx_frame[i] = c;
    i++;
     if (c == '\n' && last_c == '\r')
     {
        
        resolveRxFrame(rx_frame, &dx, &dy, &dth);
//        Serial.print("RX -> ");
//        for (int j = 0; j < 32; j++)
//        {
//          Serial.print(rx_frame[j], HEX);
//          Serial.print(" ");
//        }
//        Serial.println();
        Serial.print("Rx -> X= ");
        Serial.print(dx, 5);
        Serial.print(", Y=");
        Serial.print(dy, 5);
        Serial.print(", THETA=");
        Serial.println(dth, 5);
        i = 0;
        memset(rx_frame , 0, 32);
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
      x += dx;
      y += dy;
      theta += dtheta;
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
