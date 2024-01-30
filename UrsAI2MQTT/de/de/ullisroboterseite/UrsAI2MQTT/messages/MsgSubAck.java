package de.ullisroboterseite.UrsAI2MQTT.messages;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ SUBACK.
 */
public class MsgSubAck extends MsgMqtt {
    public byte MaxQoS;
    public boolean SubscribeFailure;
    public String Topic = ""; // Topic der zugehörigen SUBSCRIBE-Nachricht.

    /**
     * \brief Erstellt eine neue Instanz der MsgSubAck-Klasse anhand einen
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    MsgSubAck(MqttPacketBuffer mpb) {
        super(mpb);

        packetIdentifier =mpb.getByteValue()*256 + mpb.getByteValue();
        int returnCode =  mpb.getByteValue();
        MaxQoS = (byte) (returnCode & 0x03);
        SubscribeFailure = returnCode == 0x80;
    }
}