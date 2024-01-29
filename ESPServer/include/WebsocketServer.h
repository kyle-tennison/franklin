#include <WiFi.h>
#include <constants.h>
#include <shared_variables.h>

/// @brief passed to operation handlers; contains information about request
struct OperationRequest
{
  bool is_valid;
  uint8_t operation_code;
  uint8_t *payload;
  uint16_t payload_length;
  WiFiClient *client;
};

/// @brief handles websocket connections and messages
class WebsocketServer
{
public:
  /// @brief opens ESP as access point and begins WiFiServer
  /// @param s is a pointer to to the server
  void begin(WiFiServer *s)
  {
    this->server = s;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASSWORD);
    IPAddress host_ip = WiFi.softAPIP();
    Serial.print("info: opened access point with ip ");
    Serial.println(host_ip);

    server->begin();
    Serial.print("info: started server on port ");
    Serial.println(SERVER_PORT);
  }

  /// @brief waits for an incoming connection
  /// @return The WiFiClient of the incoming connection
  WiFiClient accept()
  {
    debug_print("debug: waiting for client...");
    while (1)
    {
      WiFiClient client = server->accept();
      if (client)
      {
        Serial.println("\ninfo: accepting new client");
        return client;
      }
      else
      {
        Serial.print(".");
        delay(1000);
      }
    }
  }

  /// @brief converts an incoming message into an OperationRequest
  /// @param client is a pointer to the WiFiClient who is streaming the message
  /// @return the OperationRequest constructed from the connection
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

      // if (millis() - start_time > 5000)
      // {

      //   if (read == 0)
      //   {
      //     Serial.println("error: connection timed out");
      //     client->stop();
      //     return OperationRequest{false, 0, NULL, 0, NULL};
      //   }
      //   else
      //   {
          if (!client->connected())
          {
            Serial.println("error: client lost connection");
            return OperationRequest{false, 0, NULL, 0, NULL};
          }
      //   }
      // }

      if (client->available())
      {
        uint8_t recv = client->read();

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
              debug_print("debug: determined content length to be ");
              debug_print(payload_length);
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
      else
      {
        delay(1);
      }

      if (payload_index == payload_length && payload_length_set)
      {
        // debug_print("debug: received the following payload");
        // for (uint8_t i = 0; i < payload_length; i++)
        // {
        //   Serial.print("  ");
        //   Serial.print(*(payload + i), HEX);
        // }

        return OperationRequest{true, operation, payload, payload_length, client};
      }
    }

    // loop breaks if client disconnects
    Serial.println("error: client disconnected unexpectedly");
    return OperationRequest{false, 0, NULL, 0, NULL};
  }


  /// @brief echos bytes from the client 
  /// @param operation the operation to respond to
  void echo(OperationRequest *operation)
  {

    if (operation->payload == NULL)
    {
      Serial.println("error: cannot echo NULL payload; memory error");
    }
    else
    {
      for (uint16_t i = 0; i < operation->payload_length; i++)
      {
        operation->client->write(*(operation->payload + i));
      }
    }
    Serial.println("info: echoed payload");
  }

  void status_poll(OperationRequest *operation) {

    PidState pid_state_copy;
    KinematicState kinematic_state_copy;
    MotionInfo motion_info_copy;

    if (xSemaphoreTake(pid_state_mutex, pdMS_TO_TICKS(MUTEX_MAX_WAIT)) == pdTRUE)
    {
      pid_state_copy = pid_state;
      xSemaphoreGive(pid_state_mutex);
    }
    else {
      Serial.println("error: failed to acquire PID mutex for status poll");
      return;
    }

    if (xSemaphoreTake(kinematic_state_mutex, pdMS_TO_TICKS(MUTEX_MAX_WAIT)) == pdTRUE)
    {
      kinematic_state_copy = kinematic_state;
      xSemaphoreGive(kinematic_state_mutex);
    }
    else {
      Serial.println("error: failed to acquire PID mutex for status poll");
      return;
    }

    if (xSemaphoreTake(motion_info_mutex, pdMS_TO_TICKS(MUTEX_MAX_WAIT)) == pdTRUE)
    {
      motion_info_copy = motion_info;
      xSemaphoreGive(motion_info_mutex);
    }
    else {
      Serial.println("error: failed to acquire gyro mutex for status poll");
      return;
    }

    uint8_t i = 0;
    uint16_t variables[] ={
      pid_state_copy.proportional,
      pid_state_copy.integral,
      pid_state_copy.derivative,
      ((kinematic_state_copy.motors_enabled) ? (uint8_t)1 : (uint8_t)0),
      kinematic_state_copy.gyro_offset,
      motion_info_copy.gyro_value * 100,
      motion_info_copy.integral_sum * 10,
      motion_info_copy.motor_target * 100,
    };

    uint8_t max_packet_size = 4 * sizeof(variables) / 2;

    uint8_t* payload = (uint8_t*)malloc(max_packet_size);

    uint8_t payload_index = 0;
    for (uint8_t i = 0; i < sizeof(variables) / 2; i++){
      uint16_t* var = &variables[i];

      uint8_t b0 = (*var >> 8) & 0x00FF;
      uint8_t b1 = (*var >> 0) & 0x00FF;

      *(payload + payload_index) = i;
      *(payload + payload_index + 1) = b0;
      *(payload + payload_index + 2) = b1;
      *(payload + payload_index + 3) = 0;
      payload_index += 4;
    };

    // send header
    uint8_t header [] = {70, 70, 0, payload_index, 0};
    for (uint8_t i = 0; i < sizeof(header); i++){
      operation->client->write(header[i]);
    }

    for (uint8_t i = 0; i < payload_index; i++){
      operation->client->write(*(payload + i));
    }

    free(payload);
  }


public:
  WiFiServer *server;
};

void handle_message(OperationRequest *operation);
void handle_var_update(OperationRequest *operation);
bool handle_var_update_inner(uint8_t target, uint16_t value);
void websocket_loop(void *_);

/// @brief relays general message from websocket to serial
/// @param operation is a pointer to the operation to handle
void handle_message(OperationRequest *operation)
{
  if (operation->operation_code != 0)
  {
    Serial.print("error: message must have operation code 0, found ");
    Serial.println(operation->operation_code);
    return;
  }
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

/// handles variable updates
/// @param operation is a pointer to the operation to handle
void handle_var_update(OperationRequest *operation)
{
  if (operation->operation_code != 1)
  {
    Serial.print("error: variable update must have operation code 1, found ");
    Serial.println(operation->operation_code);
    return;
  }
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
        handle_var_update_inner(target, value);
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

/// @brief inner function for updating variables. Actually makes the modification
/// @param target is the integer id of the variable to target
/// @param value is the value to assign to the targeted integer
/// @return true if the variable is successfully updated, false otherwise
bool handle_var_update_inner(uint8_t target, uint16_t value)
{

  // lock corresponding mutex
  debug_print("debug: waiting for ");

  SemaphoreHandle_t *mutex;

  if (target <= 3)
  {
    mutex = &pid_state_mutex;
    debug_print("PID Mutex");
  }
  else
  {
    mutex = &kinematic_state_mutex;
    debug_print("Kinematic Mutex");
  }

  debug_print("...");

  if (xSemaphoreTake(*mutex, pdMS_TO_TICKS(MUTEX_MAX_WAIT)) == pdTRUE)
  {
    debug_print("debug: acquired mutex");
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
  case 5:
    Serial.print("info: updating motor enabled state to ");
    Serial.println((value == 1));
    kinematic_state.motors_enabled = (value == 1);
    break;
  case 6:
    Serial.print("info: updating gyro offset to ");
    Serial.println((double)(value - 128)/10);
    kinematic_state.gyro_offset = value;
    break;
  default:
    Serial.print("error: received unknown target ");
    Serial.println(target);
  }

  xSemaphoreGive(*mutex);
  debug_print("debug: returned mutex");

  return true;
}

/// @brief monitors and handles websocket communication
/// @param _ unused
void websocket_loop(void *_)
{

  debug_print("debug: opened websocket handler");
  delay(1000);
  Serial.println("info: starting server...");
  WiFiServer server(SERVER_PORT);
  WebsocketServer sock;
  sock.begin(&server);
  debug_print("debug: instantiated server");

  WiFiClient client = sock.accept();

  while (1)
  {

    if (!client.connected())
    {
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
      debug_print("debug: dispatching to message");
      handle_message(&request);
      break;
    case 1:
      debug_print("debug: dispatching to variable update");
      handle_var_update(&request);
      break;
    case 2:
      debug_print("debug: dispatching to echo");
      sock.echo(&request);
      break;
    case 3:
      debug_print("debug: dispatching to status poll");
      sock.status_poll(&request);
      break;
    default:
      Serial.print("error: unknown operation ");
      Serial.println(request.operation_code);
    }

    free(request.payload);
  }
}