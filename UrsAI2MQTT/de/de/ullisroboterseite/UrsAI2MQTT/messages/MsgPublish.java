package de.ullisroboterseite.UrsAI2MQTT.messages;

import de.ullisroboterseite.UrsAI2MQTT.MqttClient;
import de.ullisroboterseite.UrsAI2MQTT.MqttException;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ PUBLSISH
 */
public class MsgPublish extends MsgMqtt {
    private String topic;
    private byte[] payload;
    private boolean retain;
    private int qos;
    private boolean dup;

    private static Byte buildHeaderFlags(byte qos, Boolean retain) {
        byte f = (byte) (qos << 1);
        if (retain)
            f |= 0x1;
        return f;
    }

    /**
     * \brief Erstellt eine neue Instanz der MsgPublish-Klasse.
     *
     * @param topic    Das Topic der Nachricht.
     * @param payload  Die Nutzdaten.
     * @param retained Gibt an, ob die Nachricht gepeichert werden soll.
     * @param qos      Quality of Service für diese Nachricht. Kleiner 0 wird zu 0,
     *                 größer 2 wird zu 2.
     * @throws MqttException
     */
    public MsgPublish(String topic, byte[] payload, boolean retained, byte qos) {
        super( MsgType.MQTTPUBLISH,  buildHeaderFlags(qos, retained));
        if (qos < 0)
            qos = 0;
        if (qos > 2)
            qos = 2;
        this.qos = qos;
        this.topic = topic;
        this.payload = payload;
        packet.append(topic);
        if (qos > 0) {
            packet.append((byte) (packetIdentifier >> 8));
            packet.append((byte) (packetIdentifier & 0xFF));
        }

        packet.append(payload);
    }

    /**
     * \brief Erstellt eine neue Instanz der MagPublish-Klasse anhand einen
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    MsgPublish(MqttPacketBuffer mpb) {
        super(mpb);
        int flags = mpb.getFlags();
        retain = (flags & 0x01) != 0;
        qos = (flags & 0x06) >> 1;
        dup = (flags & 0x08) != 0;

        // Topic einlesen
        int topicLen = mpb.getByteValue()*256 + mpb.getByteValue();

        byte[] tn = new byte[topicLen];
        for (int i = 0; i < topicLen; i++)
            tn[i] = mpb.getByte();
        topic = new String(tn, MqttClient.Charset);
        int payloadLen = mpb.getDataLength() - topicLen - 2;
        if (qos > 0) {
            int packetID =mpb.getByteValue() *256 + mpb.getByteValue();
            payloadLen -= 2;
            packetIdentifier = packetID;
        }

        payload = new byte[payloadLen];
        for (int i = 0; i < payloadLen; i++)
            payload[i] = mpb.getByte();
    }

    /**
     * \brief Liefert das Topic-Element der Nachricht.
     *
     * @return Das Topic-Element der Nachricht.
     */
    public String getTopic() {
        return topic;
    }

    /**
     * \brief Liefert die Nutzdaten.
     *
     * @return Die Nutzdaten als Byte-Array.
     */
    public byte[] getPayload() {
        return payload;
    }

    /**
     * \brief Liefert die Nutzdaten als String.
     *
     * @return Die Nutzdaten als String.
     */
    public String getPayloadString() {
        try {
            return new String(payload, MqttClient.Charset);
        } catch (Exception e) {
            return "";
        }

    }

    /**
     * \brief Liefert den Wert des Retain-Flags.
     *
     * @return Der Wert des Retain-Flags.
     */
    public boolean getRetainFlag() {
        return retain;
    }

    /**
     * \brief Liefert den Wert des QoS-Flags.
     *
     * @return Der Wert des QoS-Flags
     */
    public int getQoS() {
        return qos;
    }

    /**
     * \brief Setzt das DUP-Flag
     */
    public void setDupFlag() {
        dup = true;
        packet.setFlags(packet.getFlags() | 0x08);
    }

    /**
     * \brief Liefert den Wert des DUP-Flags.
     *
     * @return Der Wert des DUP-Flags.
     */
    public boolean getDupFlag() {
        return dup;
    }

    /**
     * \brief Die Nachricht wird mit gesetztem Dup-Flag zur wiederholten Sendung
     * zurück geliefert.
     */
    @Override
    public MsgMqtt retryAction() throws MqttException {
        setDupFlag();
        return this;
    }

    /**
     * \brief Gibt an, ob nach dem Versand dieser Nachricht eine Antwort vom Broker
     * erwartet wird.
     */
    @Override
    public boolean mustBeConfirmed() {
        if (qos > 0)
            return true;
        return false;
    }

    @Override
    public String toString() {
        return super.toString() + " ID: " + packetIdentifier + " QoS: " + qos + " Dup: " + dup + " Retain: " + retain
                + " Topic: " + topic;
    }
}