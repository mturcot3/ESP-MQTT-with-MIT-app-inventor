package de.ullisroboterseite.UrsAI2MQTT.messages;

import de.ullisroboterseite.UrsAI2MQTT.MqttException;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ PUBREL
 */
public class MsgPubRel extends MsgMqtt {
    public MsgPubRel(int PacketID) {
        super(MsgType.MQTTPUBREL,  (byte) 0x02);
        packetIdentifier = PacketID;
        packet.append((byte) (packetIdentifier >> 8));
        packet.append((byte) (packetIdentifier & 0xFF));
    }

    /**
     * \brief Erstellt eine neue Instanz der MsgPubRel-Klasse anhand einen
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    MsgPubRel(MqttPacketBuffer mpb) {
        super(mpb);

        packetIdentifier =mpb.getByteValue() *256 +mpb.getByteValue();// mpb.dataBuffer.byteAt(0) * 256 + mpb.dataBuffer.byteAt(1);
    }

    /**
     * \brief Die Nachricht wird zur wiederholten Sendung zurück geliefert.
     */
    @Override
    public MsgMqtt retryAction() throws MqttException {
        return this;
    }

    /**
     * \brief Gibt an, ob nach dem Versand dieser Nachricht eine Antwort vom Broker
     * erwartet wird.
     */
    @Override
    public boolean mustBeConfirmed() {
        return true;
    }

    @Override
    public String toString() {
        return super.toString() + " ID: " + packetIdentifier;
    }
}