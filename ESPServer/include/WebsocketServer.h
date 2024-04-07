#include <WiFi.h>
#include <constants.h>
#include <shared_variables.h>

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
    debug_println("debug: waiting for client...");
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

    uint8_t b1 = 0;
    uint8_t b2 = 0;

    debug_println("debug: resolving incoming request");

    while (1)
    {

      if (!client->connected())
      {
        Serial.println("error: client lost connection");
        return OperationRequest{false, 0, NULL, 0, NULL};
      if (!client->connected())
      {
        Serial.println("error: client lost connection");
        return OperationRequest{false, 0, NULL, 0, NULL};
      }

      if (client->available())
      {
        uint8_t header[5];
        client->readBytes(header, sizeof(header));
        uint8_t header[5];
        client->readBytes(header, sizeof(header));

        if (!(header[0] == header[1] && header[1] == HEADER_BYTE))
        if (!(header[0] == header[1] && header[1] == HEADER_BYTE))
        {
          Serial.println("error: invalid header byte(s): ");
          Serial.println(header[0]);
          Serial.println(header[1]);
          Serial.println("error: invalid header byte(s): ");
          Serial.println(header[0]);
          Serial.println(header[1]);
          continue;
        }

        operation = header[2];
        payload_length = header[3] << 8 | header[4];

        payload = (uint8_t *)malloc(payload_length);
        client->readBytes(payload, payload_length);

#ifdef DEBUG
        operation = header[2];
        payload_length = header[3] << 8 | header[4];

        payload = (uint8_t *)malloc(payload_length);
        client->readBytes(payload, payload_length);

#ifdef DEBUG
        debug_print("debug: received the following payload: ");
        for (uint8_t i = 0; i < payload_length; i++)
        {
          debug_print("  ");
          debug_print(*(payload + i));
        }
        debug_println();
#endif
#endif

        return OperationRequest{true, operation, payload, payload_length, client};
      }
      else
      {
        delay(1);
      }
      else
      {
        delay(1);
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
      operation->client->write(operation->payload, operation->payload_length);
      operation->client->write(operation->payload, operation->payload_length);
    }
    Serial.println("info: echoed payload");
  }

  void check_incoming_queue()
  {

    MotionInfoQueueItem incoming_item;
    MotionInfoQueueItem incoming_item;

    if (xQueueReceive(motion_to_sock_queue, &incoming_item, 0) == pdPASS)
    {
      debug_println("debug: received item from motion -> sock");
      motion_info_cache = incoming_item.motion_info;
      debug_println("debug: updated motion info cache");
    }

  }

  void status_poll(OperationRequest *operation)
  {

    check_incoming_queue();

    uint8_t i = 0;
    int16_t variables[] = {
        pid_state_cache.proportional,
        pid_state_cache.integral,
        pid_state_cache.derivative,
        ((kinematic_state_cache.motors_enabled) ? (uint8_t)1 : (uint8_t)0),
        (int16_t)(kinematic_state_cache.gyro_offset * 10.0),
        (int16_t)(motion_info_cache.gyro_value * 100.0),
        (int16_t)(motion_info_cache.integral_sum * 10.0),
        (int16_t)(motion_info_cache.motor_target * 100.0),
    };

    uint8_t max_packet_size = 4 * sizeof(variables) / 2;

    uint8_t *payload = (uint8_t *)malloc(max_packet_size);

    uint8_t payload_index = 0;
    for (uint8_t i = 0; i < sizeof(variables) / 2; i++)
    {
      int16_t *var = &variables[i];

      uint8_t b0 = (*var >> 8) & 0x00FF;
      uint8_t b1 = (*var >> 0) & 0x00FF;

      *(payload + payload_index) = i;
      *(payload + payload_index + 1) = b0;
      *(payload + payload_index + 2) = b1;
      *(payload + payload_index + 3) = 0;
      payload_index += 4;
    };

    uint8_t header[] = {70, 70, 0, payload_index, 0};
    operation->client->write(header, sizeof(header));
    operation->client->write(payload, payload_index);
    debug_print("debug: responded to poll request. content length ");
    debug_println(payload_index);
    operation->client->write(header, sizeof(header));
    operation->client->write(payload, payload_index);
    debug_print("debug: responded to poll request. content length ");
    debug_println(payload_index);

    free(payload);
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

    uint8_t target = *(operation->payload);

    uint8_t b1 = *(operation->payload + 1);
    uint8_t b2 = *(operation->payload + 2);

    int16_t value = b1 << 8 | b2;

    ConfigQueueItem update;
    update.target = UpdateTarget(target);
    update.value = value;

    if (xQueueSend(sock_to_motion_queue, &update, 0) != pdPASS)
    {
      Serial.println("warning: failed to send item update");
    }
    else
    {
      debug_println("debug: added item to motion queue");
    }


    // TODO: this is duplicate code. create a function that takes a reference
    // to an object for assignment and a reference to the "incoming item"
    //
  
    switch (update.target)
    {
    case UpdateTarget::PidProportional:
        pid_state_cache.proportional = update.value;
        debug_print("debug: updating PidProportional cache to ");
        debug_println(update.value);
        break;
    case UpdateTarget::PidDerivative:
        pid_state_cache.derivative = update.value;
        debug_print("debug: updating PidDerivative cache to ");
        debug_println(update.value);
        break;
    case UpdateTarget::PidIntegral:
        pid_state_cache.integral = update.value;
        debug_print("debug: updating PidIntegral cache to ");
        debug_println(update.value);
        break;
    case UpdateTarget::MotorsEnabled:
        kinematic_state_cache.motors_enabled = update.value == 1;
        debug_print("debug: updating MotorsEnabled cache to ");
        debug_println(update.value == 1);
        break;
    case UpdateTarget::GyroOffset:
        debug_print("raw value is:");
        debug_println(update.value);
        kinematic_state_cache.gyro_offset = (double)update.value / 10.0;
        debug_print("debug: updating GyroOffset cache to ");
        debug_println(kinematic_state_cache.gyro_offset);
        break;
    case UpdateTarget::AngularVelocityTarget:
        kinematic_state_cache.angular_velocity_target = update.value;
        debug_print("debug: updating AngularVelocityTarget cache to ");
        debug_println(update.value);
        break;
    case UpdateTarget::LinearVelocityTarget:
        kinematic_state_cache.linear_velocity_target = update.value;
        debug_print("debug: updating LinearVelocityTarget cache to ");
        debug_println(update.value);
        break;
    default:
        Serial.print("error: unable to deserialize ConfigQueueItem with target ");
        Serial.print(update.target);
        Serial.println(" in websocket loop");
        return;
    }

  }
}

private:
  WiFiServer *server;

  MotionInfo motion_info_cache;
  KinematicState kinematic_state_cache;
  PidState pid_state_cache;
};

void handle_message(OperationRequest *operation);
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

/// @brief monitors and handles websocket communication
/// @param _ unused
void websocket_loop(void *_)
{

  debug_println("debug: opened websocket handler");
  delay(1000);
  Serial.println("info: starting server...");
  WiFiServer server(SERVER_PORT);
  WebsocketServer sock;
  sock.begin(&server);
  debug_println("debug: instantiated server");

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
      debug_println("debug: dispatching to message");
      handle_message(&request);
      break;
    case 1:
      debug_println("debug: dispatching to variable update");
      sock.handle_var_update(&request);
      break;
    case 2:
      debug_println("debug: dispatching to echo");
      sock.echo(&request);
      break;
    case 3:
      debug_println("debug: dispatching to status poll");
      sock.status_poll(&request);
      break;
    default:
      Serial.print("error: unknown operation ");
      Serial.println(request.operation_code);
    }

    free(request.payload);
  }
}