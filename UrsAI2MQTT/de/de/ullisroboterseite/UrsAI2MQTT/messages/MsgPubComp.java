package de.ullisroboterseite.UrsAI2MQTT.messages;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ PUBCOMP
 */
public class MsgPubComp extends MsgMqtt {
    public MsgPubComp(int PacketID) {
        super(MsgType.MQTTPUBCOMP);
        packetIdentifier = PacketID;
        packet.append((byte) (packetIdentifier >> 8));
        packet.append((byte) (packetIdentifier & 0xFF));
    }

    /**
     * \brief Erstellt eine neue Instanz der MsgPubComp-Klasse anhand einen
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    MsgPubComp(MqttPacketBuffer mpb) {
        super(mpb);

        packetIdentifier = mpb.getByteValue() * 256 + mpb.getByteValue();// mpb.dataBuffer.byteAt(0) * 256 + mpb.dataBuffer.byteAt(1);
    }

    @Override
    public String toString() {
        return super.toString() + " ID: " + packetIdentifier;
    }
}