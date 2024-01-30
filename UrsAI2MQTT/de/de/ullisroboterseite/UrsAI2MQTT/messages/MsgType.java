package de.ullisroboterseite.UrsAI2MQTT.messages;

public enum MsgType {
    MQTTCONNECT(0x10), // Client request to connect to Server
    MQTTCONNACK(0x20), // Connect Acknowledgment

    MQTTPUBLISH(0x30), // Publish message

    // für QoS 1
    MQTTPUBACK(0x40), // Publish Acknowledgment

    // für QoS 2
    MQTTPUBREC(0x50), // Publish Received (assured delivery part 1)
    MQTTPUBREL(0x60), // Publish Release (assured delivery part 2)
    MQTTPUBCOMP(0x70), // Publish Complete (assured delivery part 3)

    MQTTSUBSCRIBE(0x80), // Client Subscribe request
    MQTTSUBACK(0x90), // Subscribe Acknowledgment

    MQTTUNSUBSCRIBE(0xA0), // Client Unsubscribe request
    MQTTUNSUBACK(0xB0), // Unsubscribe Acknowledgment

    MQTTPINGREQ(0xC0), // PING Request
    MQTTPINGRESP(0xD0), // PING Response

    MQTTDISCONNECT(0xE0); // Client is Disconnecting

    private final byte value;

    MsgType(int value) {
        this.value = (byte) value;
    }

    public byte getValue() {
        return value;
    }

  public  static MsgType fromByte(byte b) {
    for (MsgType t : MsgType.values()) {
        if(t.value == b)
        return t;
      }
      return null;
  }
}