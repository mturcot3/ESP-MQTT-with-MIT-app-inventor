package de.ullisroboterseite.UrsAI2MQTT.messages;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ SUBACK.
 */
public class MsgUnSubAck extends MsgMqtt {
    public String Topic = ""; // Topic der zugehörigen UNSUBSCRIBE-Nachricht.
    /**
     * \brief Erstellt eine neue Instanz der MsgUnSubAck-Klasse anhand einen
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    MsgUnSubAck(MqttPacketBuffer mpb) {
        super(mpb);
        packetIdentifier =mpb.getByteValue() *256 +mpb.getByteValue();// mpb.dataBuffer.byteAt(0) * 256 + mpb.dataBuffer.byteAt(1);
    }
}