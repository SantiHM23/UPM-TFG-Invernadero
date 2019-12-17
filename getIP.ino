#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
// gateway and subnet are optional:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

EthernetClient client;

void setup() {
  Serial.begin(9600);
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed DHCP");
    for(;;)
      ;
  }
  Serial.println(Ethernet.localIP());
}

void loop() {

}
