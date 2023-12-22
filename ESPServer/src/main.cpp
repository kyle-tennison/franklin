#include <Arduino.h>
#include <WiFi.h>

#define MUTEX_MAX_WAIT 10000

#define SSID "esp32"
#define PASSWORD "franklin44"
#define SERVER_PORT 80

#define HEADER_BYTE 0x46

WiFiServer server(SERVER_PORT);
void websocket_loop(void *_);

struct PidState
{
  int16_t proportional;
  int16_t derivative;
  int16_t integral;
};

struct KinematicState
{
  int16_t linear_velocity_target;
  int16_t angular_velocity_target;
};

struct OperationRequest
{
  bool is_valid;
  uint8_t operation_code;
  uint8_t* payload;
  uint16_t payload_length;
  WiFiClient* client;
};

class WebsocketServer
{
public:
  void begin()
  {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASSWORD);
    IPAddress host_ip = WiFi.softAPIP();
    Serial.print("info: opened access point with ip ");
    Serial.println(host_ip);

    server.begin();
    Serial.print("info: started server on port ");
    Serial.println(SERVER_PORT);
  }

  WiFiClient accept()
  {
    Serial.println("debug: waiting for client...");
    while (1)
    {
      WiFiClient client = server.accept();
      if (client)
      {
        Serial.print("info: accepting new client");
        return client;
      }
      else {
        Serial.print(".");
        delay(1000);
      }
    }
  }

  OperationRequest resolve_incoming(WiFiClient *client)
  {

    uint16_t read = 0;
    uint8_t operation = 0;

    uint8_t *payload = NULL;
    uint16_t payload_index = 0;
    uint16_t payload_length = 0;
    bool payload_length_set = false;

    // while (client->connected())
    uint32_t start_time = millis();
    while (1)
    {

      if (millis() - start_time > 10E3 && read != 0){
        Serial.println("error: connection timed out");
        client->stop();
        return OperationRequest{false, 0, NULL, 0, NULL};
      }

      if (client->available())
      {
        uint8_t recv = client->read();
        // Serial.print("recv: ");
        // Serial.print(recv);
        // Serial.print(" @ ");
        // Serial.print(read);

        if (read < 2 && recv != HEADER_BYTE)
        {
          Serial.println("error: expected header byte");
          continue;
        }
        else if (read == 2)
        {
          operation = recv;
        }
        else if (read == 3)
        {
          payload_length = recv;
        }
        else
        {
          if (!payload_length_set)
          {
            if (recv != 0)
            {
              payload_length += recv;
            }
            else
            {
              Serial.print("debug: determined content length to be ");
              Serial.println(payload_length);
              payload_length_set = true;
              payload = (uint8_t *)malloc(payload_length);
            }
          }
          else
          {
            if (payload == NULL)
            {
              Serial.println("error: payload pointer is null; memory error");
            }
            else
            {
              *(payload + payload_index) = recv;
              payload_index++;
            }
          }
        }
        read++;
      }
      else {
        delay(1);
      }

      if (payload_index == payload_length && payload_length_set)
      {
        Serial.println("info: received the following payload");
        for (uint8_t i = 0; i < payload_length; i++)
        {
          Serial.print("  ");
          Serial.print(*(payload + i), HEX);
        }

        return OperationRequest{true, operation, payload, payload_length, client};
      }
    }

    // loop breaks if client disconnects
    Serial.println("error: client disconnected unexpectedly");
    return OperationRequest{false, 0, NULL, 0, NULL};
  }

  // echo a request back to the client
  void echo(OperationRequest* operation){

    if (operation->payload == NULL) {
      Serial.println("error: cannot echo NULL payload; memory error");
    }
    else {
      for (uint16_t i = 0; i < operation->payload_length; i++){
        operation->client->write(*(operation->payload + i));
      }
    }
    Serial.println("info: echoed payload");

  }
};

void handle_message(OperationRequest *operation);

PidState pid_state;
SemaphoreHandle_t pid_state_mutex = NULL;

KinematicState kinematic_state;
SemaphoreHandle_t kinematic_state_mutex = NULL;

void setup()
{
  Serial.begin(115200);

  delay(2000);

  pid_state_mutex = xSemaphoreCreateMutex();
  pid_state.proportional = 0;
  pid_state.integral = 0;
  pid_state.derivative = 0;

  kinematic_state_mutex = xSemaphoreCreateMutex();
  kinematic_state.linear_velocity_target = 40;
  kinematic_state.angular_velocity_target = 0;

  xTaskCreatePinnedToCore(
      websocket_loop,
      "Websocket Loop",
      2048,
      NULL,
      100,
      NULL,
      0);
}

void loop()
{
  delay(1);
}

// Relays general message from websocket to serial
void handle_message(OperationRequest *operation)
{
  if (operation->payload == NULL)
  {
    Serial.println("error: operation payload pointer null; memory error");
  }
  else
  {
    Serial.print("\ninfo: received incoming message: ");
    for (uint16_t i = 0; i < operation->payload_length; i++)
    {
      Serial.print((char)*(operation->payload + i));
    }
    Serial.println();
    Serial.println();
  }
}

// Handles variable updates
void handle_var_update(OperationRequest *operation)
{
  if (operation->payload == NULL)
  {
    Serial.println("error: operation payload pointer null; memory error");
  }
  else
  {

    uint8_t target = UINT8_MAX;
    uint16_t value = 0;

    for (uint16_t i = 0; i < operation->payload_length; i++)
    {

      uint8_t recv = *(operation->payload + i);

      if (target == UINT8_MAX)
      {
        target = recv;
      }
      else if (recv == 0)
      {
        // handle update
        target = UINT8_MAX;
        value = 0;
      }
      else
      {
        value += recv;
      }
    }
  }
}

bool handle_var_update_inner(uint8_t target, uint16_t value)
{

  // lock corresponding mutex
  Serial.print("debug: waiting for ");

  SemaphoreHandle_t *mutex;

  if (target <= 3)
  {
    mutex = &pid_state_mutex;
    Serial.print("PID Mutex");
  }
  else
  {
    mutex = &kinematic_state_mutex;
    Serial.print("Kinematic Mutex");
  }

  Serial.println("...");

  if (xSemaphoreTake(*mutex, pdMS_TO_TICKS(MUTEX_MAX_WAIT)) == pdTRUE)
  {
    Serial.println("debug: acquired mutex");
  }
  else
  {
    Serial.println("error: timed out waiting for mutex");
    return false;
  }

  switch (target)
  {
  case 0:
    Serial.print("info: updating PID proportional to ");
    Serial.println(value);
    pid_state.proportional = value;
    break;
  case 1:
    Serial.print("info: updating PID integral to ");
    Serial.println(value);
    pid_state.integral = value;
    break;
  case 2:
    Serial.print("info: updating PID derivative to ");
    Serial.println(value);
    pid_state.derivative = value;
    break;
  case 3:
    Serial.print("info: updating linear velocity target to ");
    Serial.println(value);
    kinematic_state.linear_velocity_target = value;
    break;
  case 4:
    Serial.print("info: updating angular velocity target to ");
    Serial.println(value);
    kinematic_state.angular_velocity_target = value;
    break;
  default:
    Serial.print("error: received unknown target ");
    Serial.println(target);
  }

  xSemaphoreGive(*mutex);
  Serial.println("debug: returned mutex");

  return true;
}

// Loop to monitor incoming connections
void websocket_loop(void *_)
{
  WebsocketServer sock;
  sock.begin();

  WiFiClient client = sock.accept();

  while (1)
  {

    if (!client.connected()){
      client.stop();
      Serial.println("info: client closed");
      client = sock.accept();
    }

    OperationRequest request = sock.resolve_incoming(&client);

    if (!request.is_valid)
    {
      Serial.println("error: failed to resolve request, continuing");
      continue;
    }

    // dispatch request
    switch (request.operation_code)
    {
    case 0:
      Serial.println("debug: dispatching to message");
      handle_message(&request);
      break;
    case 1:
      Serial.println("debug: dispatching to variable update");
      handle_var_update(&request);
      break;
    case 2:
      Serial.println("debug: dispatching to echo");
      sock.echo(&request);
      break;
    default:
      Serial.print("error: unknown operation ");
      Serial.println(request.operation_code);
    }

    free(request.payload);
  }
}