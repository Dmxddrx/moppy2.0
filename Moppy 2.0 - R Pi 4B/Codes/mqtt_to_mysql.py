import json
import mysql.connector
import paho.mqtt.client as mqtt

# MySQL Configuration
DB_CONFIG = {
    'host': 'localhost',
    'user': 'root',
    'password': 'YOUR_MYSQL_PASSWORD', 
    'database': 'climate_db'
}

# MQTT Configuration
MQTT_HOST = "127.0.0.1"
MQTT_PORT = 1883
MQTT_USER = "climateuser"
MQTT_PASS = "CR#i22@53N"
MQTT_TOPIC = "sensor/data"

def on_connect(client, userdata, flags, rc):
    print(f"Connected to Mosquitto with result code {rc}")
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    payload = msg.payload.decode('utf-8')
    print(f"Received: {payload}")
    
    try:
        data = json.loads(payload)
        
        # Verify this is a ClimateNode payload
        if data.get("type") == "ENV":
            
            # Open DB connection
            conn = mysql.connector.connect(**DB_CONFIG)
            cursor = conn.cursor()
            
            # Prepare the INSERT statement
            sql = """INSERT INTO ClimateLogs 
                     (Device, Temperature, Humidity, Dust, RawAdc, RawVolt, Status, Timestamp) 
                     VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"""
            
            # Extract data mapping exactly to your C++ JSON structure
            val = (
                data.get("device_id"),
                data.get("temp"),
                data.get("hum"),
                data.get("dust"),
                data.get("raw_adc"),
                data.get("raw_volt"),
                data.get("status"),
                data.get("timestamp")
            )
            
            # Execute and save
            cursor.execute(sql, val)
            conn.commit()
            print(f"Saved {cursor.rowcount} row(s) to MySQL.")
            
            # Clean up
            cursor.close()
            conn.close()
            
    except Exception as e:
        print(f"Error processing message: {e}")

# Initialize and run the MQTT client
client = mqtt.Client()
client.username_pw_set(MQTT_USER, MQTT_PASS)
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_HOST, MQTT_PORT, 60)
print("MQTT to MySQL ingestion started...")
client.loop_forever()
