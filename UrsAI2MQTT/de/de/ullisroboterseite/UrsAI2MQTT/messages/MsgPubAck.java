package de.ullisroboterseite.UrsAI2MQTT.messages;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ PUBACK
 */
public class MsgPubAck extends MsgMqtt {
    public MsgPubAck(int PacketID) {
        super(MsgType.MQTTPUBACK);
        packetIdentifier = PacketID;
        packet.append((byte) (packetIdentifier >> 8));
        packet.append((byte) (packetIdentifier & 0xFF));
    }

    /**
     * \brief Erstellt eine neue Instanz der MsgPubAck-Klasse anhand einen
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    MsgPubAck(MqttPacketBuffer mpb) {
        super(mpb);

        packetIdentifier = mpb.getByteValue() *256 + mpb.getByteValue();
    }

    @Override
    public String toString() {
        return super.toString() + " ID: " + packetIdentifier;
    }
}