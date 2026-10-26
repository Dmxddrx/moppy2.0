import socket
import paho.mqtt.publish as publish

# Server configuration
TCP_IP = '0.0.0.0'
TCP_PORT = 8080
MQTT_HOST = '127.0.0.1'
MQTT_PORT = 1883
MQTT_AUTH = {'username':"climateuser", 'password':"CR#i22@53N"}

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((TCP_IP, TCP_PORT))
s.listen(5)
print("TCP-to-MQTT Bridge listening on port 8080...")

while True:
    conn, addr = s.accept()
    print(f"Node connected: {addr}")
    while True:
        try:
            data = conn.recv(1024)
            if not data:
                break # Connection dropped by STM32
            
            payload = data.decode('utf-8').strip()
            if payload:
                print(f"Routing data: {payload}")
                publish.single("sensor/data", payload, hostname=MQTT_HOST, port=MQTT_PORT, auth=MQTT_AUTH)
        except Exception as e:
            print(f"Error: {e}")
            break
    print("Node disconnected.")
    conn.close()
