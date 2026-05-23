#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
from binascii import unhexlify

from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from gurux_dlms.GXDLMSTranslator import GXDLMSTranslator

import paho.mqtt.client as mqtt

# --------------------------------------------------
# KONFIGURATION
# --------------------------------------------------

PORT = "/dev/ttyS0"
BAUDRATE = 2400

SMARTMETER_KEY = "KEY_FROM_YOUR_EVU"

MQTT_HOST = "SERVER_ADRESS"
MQTT_PORT = 1883
MQTT_USER = "MQTT_USER"
MQTT_PASSWORD = "MQTT_PASS"

# --------------------------------------------------

translator = GXDLMSTranslator()

segment0 = None

# --------------------------------------------------
# MQTT
# --------------------------------------------------

mqtt_client = mqtt.Client(
    client_id="HoneywellDM515")

mqtt_client.username_pw_set(
    MQTT_USER,
    MQTT_PASSWORD
)

mqtt_client.connect(
    MQTT_HOST,
    MQTT_PORT,
    60
)

mqtt_client.loop_start()

# --------------------------------------------------
# OBIS MAP
# --------------------------------------------------

OBIS_MAP = {

    "0100200700FF": ("voltage_l1", 0.1),
    "0100340700FF": ("voltage_l2", 0.1),
    "0100480700FF": ("voltage_l3", 0.1),

    "01001F0700FF": ("current_l1", 0.01),
    "0100330700FF": ("current_l2", 0.01),
    "0100470700FF": ("current_l3", 0.01),

    "0100010700FF": ("power_import", 1.0),
    "0100020700FF": ("power_export", 1.0),

    "0100010800FF": ("energy_import", 0.001),
    "0100020800FF": ("energy_export", 0.001),

    "0100030800FF": ("reactive_import", 0.001),
    "0100040800FF": ("reactive_export", 0.001),
}

# --------------------------------------------------
# MQTT Publish
# --------------------------------------------------

def publish_value(topic, value):

    mqtt_client.publish(
        f"smartmeter/{topic}",
        value,
        retain=True
    )

# --------------------------------------------------
# FRAME LESEN
# --------------------------------------------------

def read_frame(ser):

    while True:

        b = ser.read(1)

        if not b:
            continue

        if b[0] != 0x68:
            continue

        l1 = ser.read(1)
        l2 = ser.read(1)
        s2 = ser.read(1)

        if len(l1) != 1:
            continue

        if len(l2) != 1:
            continue

        if len(s2) != 1:
            continue

        if s2[0] != 0x68:
            continue

        if l1[0] != l2[0]:
            continue

        frame_len = l1[0]

        rest = ser.read(frame_len + 2)

        if len(rest) != frame_len + 2:
            continue

        frame = b + l1 + l2 + s2 + rest

        if frame[-1] != 0x16:
            continue

        return frame

# --------------------------------------------------
# XML PARSER
# --------------------------------------------------

def extract_values(xml):

    values = {}

    current_obis = None

    for line in xml.splitlines():

        line = line.strip()

        if '<OctetString Value="' in line:

            start = line.find('Value="') + 7
            end = line.find('"', start)

            value = line[start:end].upper()

            if value in OBIS_MAP:
                current_obis = value

        elif current_obis and "UInt16 Value=" in line:

            start = line.find('Value="') + 7
            end = line.find('"', start)

            raw = int(line[start:end], 16)

            topic, factor = OBIS_MAP[current_obis]

            values[topic] = round(raw * factor, 3)

            current_obis = None

        elif current_obis and "UInt32 Value=" in line:

            start = line.find('Value="') + 7
            end = line.find('"', start)

            raw = int(line[start:end], 16)

            topic, factor = OBIS_MAP[current_obis]

            values[topic] = round(raw * factor, 3)

            current_obis = None

    return values

# --------------------------------------------------
# SEGMENT DECODE
# --------------------------------------------------

def decode_segments(seg0_hex, seg1_hex):

    try:

        payload0 = seg0_hex[18:-4]
        payload1 = seg1_hex[18:-4]

        if not payload0.startswith("db08"):
            return

        system_title = payload0[4:20]
        frame_counter = payload0[28:36]

        cipher0 = payload0[36:]

        cipher_complete = cipher0 + payload1

        iv = unhexlify(
            system_title +
            frame_counter
        )

        aes = AESGCM(
            unhexlify(SMARTMETER_KEY)
        )

        cipher_bytes = unhexlify(
            cipher_complete
        )

        apdu_hex = aes.encrypt(
            iv,
            cipher_bytes,
            b"0"
        ).hex()

        xml = translator.pduToXml(
            apdu_hex[:-32]
        )

        values = extract_values(xml)

        for topic, value in values.items():
            publish_value(topic, value)

        if values:

            print(
                f"P+ {values.get('power_import',0)} W | "
                f"P- {values.get('power_export',0)} W | "
                f"A+ {values.get('energy_import',0)} kWh | "
                f"A- {values.get('energy_export',0)} kWh"
            )

    except Exception as e:

        print("Decode Fehler:", e)

# --------------------------------------------------
# FRAME PROCESSING
# --------------------------------------------------

def process_frame(frame):

    global segment0

    ci = frame[6]

    if ci == 0x00:

        segment0 = frame.hex()

    elif ci == 0x11:

        if segment0:

            decode_segments(
                segment0,
                frame.hex()
            )

            segment0 = None

# --------------------------------------------------
# MAIN
# --------------------------------------------------

def main():

    print("=" * 60)
    print("HONEYWELL DM515 MQTT DECODER")
    print("=" * 60)

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=5
    )

    try:

        while True:

            frame = read_frame(ser)

            process_frame(frame)

    except KeyboardInterrupt:

        print("\nBeendet")

    finally:

        ser.close()

        mqtt_client.loop_stop()
        mqtt_client.disconnect()

# --------------------------------------------------

if __name__ == "__main__":
    main()
