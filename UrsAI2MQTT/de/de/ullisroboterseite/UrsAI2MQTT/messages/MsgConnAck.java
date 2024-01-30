package de.ullisroboterseite.UrsAI2MQTT.messages;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ CONNACK
 */
public class MsgConnAck extends MsgMqtt {
    public final boolean sessionPresent; ///< \brief true, wenn eine Session vorhanden ist.

    /**
     * \brief Return-Code der Verbindungsanforderung.
     *
     * 0 Connection accepted <br>
     * 1 Connection refused, unacceptable protocol version. The Server does not
     * support the level of the MQTT protocol requested by the Client <br>
     * 2 Connection refused, identifier rejected. The Client identifier is correct
     * UTF-8 but not allowed by the Server <br>
     * 3 Connection refused, Server unavailable. The Network Connection has been made
     * but the MQTT service is unavailable <br>
     * 4 Connection refused, bad user name or password. The data in the user name or
     * password is malformed <br>
     * 5 Connection refused, not authorized. The Client is not authorized to connect
     *
     */
    public final int connectReturnCode;

    /**
     * \brief Erstellt eine neue Instanz der Klasse \ref MsgConnAck anhand eines
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    public MsgConnAck(MqttPacketBuffer mpb) {
        super(mpb);

        int connectAcknowledgeFlags = mpb.getByteValue();
        sessionPresent = (connectAcknowledgeFlags & 0x01) != 0;
        connectReturnCode = mpb.getByteValue();
    }
}