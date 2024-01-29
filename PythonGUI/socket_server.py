from dataclasses import dataclass
import json
from pprint import pprint
import socket
import time
from typing import Optional
from pydantic import BaseModel, Field
import csv

LOGFILE = "log.csv"

@dataclass
class OperationRequest:
    code: int
    content_length: int
    content: bytes

class StatusResponse(BaseModel):
    pid_integral: float = Field(alias="PID Integral", default=0)
    pid_derivative: float = Field(alias="PID Derivative", default=0)
    pid_proportional: float = Field(alias="PID Proportional", default=0)
    integral_sum: float = Field(alias="Integral Sum", default=0)
    motor_target: float = Field(alias="Motor Target", default=0)
    gyro_offset: float = Field(alias="Gyro Offset", default=0)
    gyro_value: float = Field(alias="Gyro Value", default=0)
    motors_enabled: bool = Field(alias="Motors Enabled", default=0)


class Server:

    def __init__(self) -> None:
        self.sock = socket.socket()

        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("localhost", 2999))
        self.connect()

    def connect(self):
        print("waiting for connection...")
        self.sock.listen(1)
        self.con, addr = self.sock.accept()
        print("made connection")
        self.has_alive_connection = True

    def accept_incoming(self) -> Optional[OperationRequest]:
        recv = b''
        while recv == b'':
            recv = self.con.recv(1)

        if recv == b'F':

            header = self.con.recv(4)

            if header[0] != 70:
                print(f"error: bad header; {int(header[0])} != 70")
                return
            
            operation = header[1]

            c1 = header[2]
            c2 = header[3]

            content_length = (c1 << 8) + c2

            print("debug: content length is", content_length)

            content = self.con.recv(content_length)

            return OperationRequest(
                code=operation,
                content_length=content_length,
                content=content
            )
        
        else:
            # keep looking for a header byte
            return
        
        
    def dispatch_operation(self, operation_request: OperationRequest):

        if operation_request.code == 0:
            print(f"info: incoming message -- '{operation_request.content.decode('utf-8')}'")

        elif operation_request.code == 1:
            self.con.close()
            self.has_alive_connection = False
            print("closed connection")

        elif operation_request.code == 2:
            print("incoming json")
            incoming = StatusResponse(**json.loads(operation_request.content.decode("utf-8")))
            self.log_status(incoming)
            print(incoming.model_dump_json(indent=4))
            return incoming

        else:
            print(f"error: unknown operation {operation_request.code}")

    def log_status(self, incoming: StatusResponse):

        has_headers = True
        with open(LOGFILE, 'r') as f:
            if "motor_target" not in f.read():
                has_headers = False

        dict_buffer = incoming.model_dump()

        with open(LOGFILE, 'a') as f:
            w = csv.DictWriter(f, dict_buffer.keys())
            if not has_headers:
                w.writeheader()
            w.writerow(dict_buffer)


    def wait_for_update(self) -> StatusResponse:
        print("waiting for update...")
        while True:
            while self.has_alive_connection:
                operation = self.accept_incoming()
                if operation is None:
                    continue
                result = self.dispatch_operation(operation)

                if operation.code == 2 and isinstance(result, StatusResponse):
                    return result

            self.connect()




if __name__ == "__main__":
    server = Server()
    try:
        while True:
            while server.has_alive_connection:
                operation = server.accept_incoming()
                if operation is None:
                    continue
                server.dispatch_operation(operation)
            server.connect()

    except:
        print("closing socket")
        server.con.close()
        server.sock.close()
        raise

