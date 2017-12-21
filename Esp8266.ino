#include <ESP8266WiFi.h>

const char* ssid = "timeout";//type your ssid
const char* password = "96478588";//type your password
WiFiServer server(80);//Service Port

void setup() {
  Serial.begin(115200);
  delay(10);
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

void loop() {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Wait until the client sends some data
  //Serial.println("new client");
  while (!client.available()) {
    delay(1);
  }

  // Read the first line of the request
  String request = client.readStringUntil(' H');
  request = request.substring(5);
  Serial.println(request);
  //Serial.println(request.substring(0, 6));
  client.flush();
  // wait for answer from STM32
  while(!Serial.available()){
    delay(1);
  }
  if (Serial.available()){
    request = Serial.readString();
  }
  // Return the response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  do not forget this one
  //client.println("<!DOCTYPE HTML>");
  client.println(request);
  //client.println("<html>");


  delay(1); 
  //Serial.println("Client disconnected");
  Serial.println("");
}
