package de.ullisroboterseite.UrsAI2MQTT.messages;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ DISCONNECT
 */
public class MsgDisconnect extends MsgMqtt {
    /**
     * \brief Initializiert eine neue Instanz der MsgDisconnect-Klasse.
     */
    public MsgDisconnect() {
        super(MsgType.MQTTDISCONNECT);
    }
}