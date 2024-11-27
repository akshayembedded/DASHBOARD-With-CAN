#include <SoftwareSerial.h>

#include <SPI.h>
#include <mcp2515.h>


SoftwareSerial nextion(2, 3); // RX, TX
struct can_frame canMsg;
MCP2515 mcp2515(10);
int indicator;


void setup() {
  Serial.begin(115200);
  nextion.begin(9600); 
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
  updateVariable("va1", 3); // Call the function to update va1
      updateVariable("va0", 0); 
}
unsigned long a;
int f=0;
void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
   a=(canMsg.can_id);
    //Serial.print(canMsg.can_id);
   if(a==0x10)
   {
      float tempfloat =(float)((canMsg.data[0]<<8)+canMsg.data[1]);
      float temp =(float)((canMsg.data[2]<<8)+(canMsg.data[3]))/1000;
      //Serial.print("Temperature ");
      Serial.println(tempfloat+temp);
      //int gaugeAngle = map((int)(tempfloat+temp), 50, 140, 0, 90); // Map input to angle
      //if(gaugeAngle<0)
      //gaugeAngle=0;
      updateVariable("z0", (int)(tempfloat+temp));
      //Serial.println(gaugeAngle);
      // float hum =(float)((canMsg.data[4]<<8)+canMsg.data[5]);
      // float humfloat =(float)((canMsg.data[6]<<8)+(canMsg.data[7]))/1000;
      // Serial.print("Humidity ");
      // Serial.println(hum+humfloat);
     }
      else if(a==0x11)
      {
      // float pres =(float)((canMsg.data[0]<<8)+canMsg.data[1]);
      // float presfloat =(float)((canMsg.data[2]<<8)+(canMsg.data[3]))/1000;
      // Serial.print("Preassure ");
      // Serial.println(pres+presfloat);

     }
     

    
    // for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
    //   Serial.print(canMsg.data[i],HEX);
    //   Serial.print(" ");
    // }

    Serial.println();      
  }
  if(canMsg.can_id==3)
     {
     
        indicator=canMsg.data[0];//right
        
      }
      
     
      if(indicator)
      {
        f=1;
        updateVariable("va1", indicator); // Call the function to update va1
      updateVariable("va0", 1);
      delay(500); // Wait for a second before updating again
      updateVariable("va0", 0); // Call the function to update va1
      delay(504); // Wait for a second before updating again
      }
     else
     {
      if(f==1)
      {
      updateVariable("va1", 3); // Call the function to update va1
      updateVariable("va0", 0); // Call the function to update va1
      delay(504); // Wait for a second before updating again
      f=0;
      }
     }
}
+



void updateVariable(String variable, int value) {
  String command = variable + ".val=" + String(value);
  sendCommand(command);
}

// Function to send a command to the Nextion display
void sendCommand(String command) {
 //                Serial.println(command);
  nextion.print(command); // Send the command
  nextion.write(0xFF);    // End of command (3 bytes of 0xFF)
  nextion.write(0xFF);
  nextion.write(0xFF);
}