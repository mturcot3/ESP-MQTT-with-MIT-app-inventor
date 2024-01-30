package de.ullisroboterseite.UrsAI2MQTT.messages;

import de.ullisroboterseite.UrsAI2MQTT.MqttException;

/**
 * \brief Basis-Klasse für MQTT-Nachrichten.
 */
public class MsgMqtt {
    /** \brief Statischer Zähler für die Paket-ID. */
    protected static int nextPacketIdentifier = 1;
    /** \brief Paket-Puffer für diese Nachricht. */
    protected MqttPacketBuffer packet;
    /** \brief Paket-ID für diese Nachricht. */
    public int packetIdentifier;

    /** \brief Zeitpunkt lfd. Sekunden, zu dem diese Nachricht versand wurde. */
    public long messageSentAt = 0;

    protected MsgType msgType;

    public String getTypeName() {
        switch (getType()) {
        case MQTTCONNECT:
            return "CONNECT";
        case MQTTCONNACK:
            return "CONNACK";
        case MQTTPUBLISH:
            return "PUBLISH";
        case MQTTPUBACK:
            return "PUBACK";
        case MQTTPUBREC:
            return "PUBREC";
        case MQTTPUBREL:
            return "PUBREL";
        case MQTTPUBCOMP:
            return "PUBCOMP";
        case MQTTSUBSCRIBE:
            return "SUBSCRIBE";
        case MQTTSUBACK:
            return "SUBACK";
        case MQTTUNSUBSCRIBE:
            return "UNSUBSCRIBE";
        case MQTTUNSUBACK:
            return "UNSUBACK";
        case MQTTPINGREQ:
            return "PINGREQ";
        case MQTTPINGRESP:
            return "PINGRESP";
        case MQTTDISCONNECT:
            return "DISCONNECT";
        default:
            return "Ungültig";
        }
    }
    // #endregion

    /**
     * \brief Erstellt die Basis für eine neue Nachricht.
     *
     * @param mqqtMsgType Type der Nachricht.
     *
     */
    public MsgMqtt(MsgType mqqtMsgType) {
        this(mqqtMsgType, (byte) 0);
    }

    public MsgMqtt(MsgType mqqtMsgType, byte flags) {
        msgType = mqqtMsgType;
        packet = new MqttPacketBuffer((byte) (mqqtMsgType.getValue() | (flags & 0x0F)));
        nextPacketIdentifier++;
        if (nextPacketIdentifier > 32000)
            nextPacketIdentifier = 0;
        packetIdentifier = nextPacketIdentifier;
    }

    /**
     * \brief Erstellt die Basis für eine neue Nachricht anhand eines empfangenen
     * Datenpakets.
     *
     * @param mpb Das empfangene Datenpaket.
     */
    public MsgMqtt(MqttPacketBuffer mpb) {
        packet = mpb;
    }

    /**
     * \brief Liefert den kompletten Byte-Puffer für diese Nachricht.
     *
     * @return Der kompletten Byte-Puffer für diese Nachricht.
     * @note Wird vom Network-Client zum Nachrichtenversand benötigt.
     */
    public byte[] getRawBuffer() {
        return packet.getMqttPacketData();
    }

    /**
     * Liefert den Paket-Typ dieser Nachricht.
     *
     * @return Der Paket-Typ dieser Nachricht.
     */
    public MsgType getType() {
        return (msgType);
    }

    /**
     * \brief Stößt die Aktion an, die bei einem Timeout ausgelöst werden soll.
     * \note Muss von den entsprechenden abgeleiteten Klassen überschrieben werden.
     * \note Setzt ggf. das DUP-Flag.
     *
     * @return Folgenachricht, die versendet werden soll oder null, wenn keine Nachricht versandt werden soll.
     * @throws MqttException Wenn keine interne Lösung für das Timeout gefunden
     *                       werden kann.
     */
    public MsgMqtt retryAction() throws MqttException {
        return null;
    }

    /**
     * \brief Gibt an, ob nach dem Versand dieser Nachricht eine Antwort vom Broker
     * erwartet wird.
     *
     * @return true, wenn nach dem Versand dieser Nachricht eine Antwort vom Broker
     *         erwartet wird.
     */
    public boolean mustBeConfirmed() {
        return false;
    }

    /**
     * Ertstellt eine MQTT-Nachricht aus einem empfagenen Paket.
     *
     * @param mpb Das empfangene Nachrichtenpaket.
     * @return Die aufbereitete Nachricht.
     */
    public static MsgMqtt fromBuffer(MqttPacketBuffer mpb) {
        MsgMqtt msg = null;
        MsgType t = MsgType.fromByte(mpb.getType());
        switch (t) {
        case MQTTCONNACK:
            msg = new MsgConnAck(mpb);
            break;
        case MQTTPINGREQ:
            msg = new MsgPingRequest(mpb);
            break;
        case MQTTPINGRESP:
            msg = new MsgPingResponse(mpb);
            break;
        case MQTTPUBLISH:
            msg = new MsgPublish(mpb);
            break;
        case MQTTSUBACK:
            msg = new MsgSubAck(mpb);
            break;
        case MQTTPUBACK:
            msg = new MsgPubAck(mpb);
            break;
        case MQTTPUBREC:
            msg = new MsgPubRec(mpb);
            break;
        case MQTTPUBREL:
            msg = new MsgPubRel(mpb);
            break;
        case MQTTPUBCOMP:
            msg = new MsgPubComp(mpb);
            break;
        case MQTTUNSUBACK:
            msg = new MsgUnSubAck(mpb);
            break;
        default:
            break;
        }
        msg.msgType = t;
        return msg;
    }

    @Override
    public String toString() {
        return getTypeName() + " (" + String.format("0x%2X", getType().getValue()) + ")";
    }
}