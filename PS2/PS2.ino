#include <PS2X_lib.h>  //for v1.6
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>

#define PS2_DAT        13  //14    
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        12  //17


#define pressures   false
#define rumble      false

PS2X ps2x; // create PS2 Controller Class
ros::NodeHandle node_handle;

int error = 0;
byte type = 0;
byte vibrate = 0;

// parameters for reading the joystick:
int range = 127;               // output range of X or Y movement
int responseDelay = 5;        // response delay of the mouse, in ms
int threshold = 2;      // resting threshold
int center = 127;         // resting position value

sensor_msgs::JointState msg;
std_msgs::Int8 msg_;
ros::Publisher pub("joystick", &msg);

char* id = "/joint";
char *a[] = {"FL", "FR", "BR", "BL","BA","BC"};
float pos[6];

void setup(){
 
  Serial.begin(57600);
  
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  node_handle.initNode();
  node_handle.advertise(pub);

  
  msg.header.frame_id = id;
  msg.name_length = 6;
  msg.position_length = 6;
  msg.name= a;
  msg.position = pos;
  
}

int x_ = 0;
int y_ = 0;
int z_ = 0;
int rx_ = 0;
int ry_ = 0;

void loop() {

  if(error == 1) //skip loop if no controller found
    return; 
  
   //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    // if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
    //   Serial.println("Start is being held");
    // if(ps2x.Button(PSB_SELECT))
    //   Serial.println("Select is being held");      

    if(ps2x.Button(PSB_PAD_LEFT)){
      // Serial.print("LEFT held this hard: ");
      // Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);                  //-x
      x_ = -1;
    }
    else if(ps2x.Button(PSB_PAD_RIGHT)){
      // Serial.print("Right held this hard: ");
      // Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);                 //+x
      x_ = 1;
    }
    else{
      x_ = 0;
    }
    
    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      // Serial.print("Up held this hard: ");
      // Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);                    //-y
      y_ = -1;
    }
    else if(ps2x.Button(PSB_PAD_DOWN)){
      // Serial.print("DOWN held this hard: ");
      // Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);                  //+y
      y_ = +1;
    }  
    else{
      y_ = 0;
    }
     
    if(ps2x.Button(PSB_L2)){
      // Serial.println("L2 pressed");                                   //-z
      z_ = -1;
    }
    else if(ps2x.Button(PSB_L1)){
        // Serial.println("L1 pressed");                                   //+z
        z_ = +1;
    }
    else{
        z_ = 0;
    }
    

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      // if(ps2x.Button(PSB_L3))
      //   Serial.println("L3 pressed");
      // if(ps2x.Button(PSB_R3))
      //   Serial.println("R3 pressed");
      // // if(ps2x.Button(PSB_L2))
      //   // Serial.println("L2 pressed");
      // if(ps2x.Button(PSB_R2))
      //   Serial.println("R2 pressed");
      // if(ps2x.Button(PSB_TRIANGLE))
      //   Serial.println("Triangle pressed");        
    }

    // if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
    //   Serial.println("Circle just pressed");
    // if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
    //   Serial.println("X just changed");
    // if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
    //   Serial.println("Square just released");     

    // if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
    //   Serial.print("Stick Values:");
    //   Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
    //   Serial.print(",");
    //   Serial.print(ps2x.Analog(PSS_LX), DEC); 
    //   Serial.print(",");
    //   Serial.print(ps2x.Analog(PSS_RY), DEC); 
    //   Serial.print(",");
    //   Serial.println(ps2x.Analog(PSS_RX), DEC); 
    // }


    int xReading = readAxis(PSS_LX);
    int yReading = readAxis(PSS_LY); 
    // if(abs(xReading)>0)
    //     Serial.println(xReading, DEC);
    // if(abs(yReading)>0)
    //     Serial.println(yReading, DEC);   

    rx_ = readAxis(PSS_RX);
    ry_ = readAxis(PSS_RY); 

    // if(abs(rx_)>0)
    //     Serial.println(rx_, DEC);
    // if(abs(ry_)>0)
    //     Serial.println(ry_, DEC);

    // Serial.println(x_, DEC); 

  msg.position[0] = x_; 
  msg.position[1] = y_; 
  msg.position[2] = z_; 
  msg.position[3] = xReading; 
  msg.position[4] = yReading;
  msg.position[5] = rx_; 

  // msg.header.stamp = ros::Time::now();
  
  pub.publish( &msg );

  node_handle.spinOnce();
  delay(50);  
}

int readAxis(int thisAxis) { 
  // read the analog input:
  int reading = ps2x.Analog(thisAxis);

  // map the reading from the analog input range to the output range:
  // reading = map(reading, 0, 255, 0, range);

  // if the output reading is outside from the
  // rest position threshold,  use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  } 

  // return the distance for this axis:
  return distance;
}
