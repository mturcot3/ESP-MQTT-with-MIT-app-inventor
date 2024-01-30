package de.ullisroboterseite.UrsAI2MQTT.messages;

import de.ullisroboterseite.UrsAI2MQTT.MqttException;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ SUBSCRIBE
 */
public class MsgSubscribe extends MsgMqtt {
    public String Topic;

    /**
     * \brief Erstellt eine neue Instanz der MsgSubscribe-Klasse.
     *
     * @param topic Der Wert für das Topic-Element.
     * @param qos   Quality of Service für die mit dieser Nachricht abonnierten
     *              Topics. Kleiner 0 wird zu 0, größer 2 wird zu 2.
     */
    public MsgSubscribe(String topic, byte qos) {
        super(MsgType.MQTTSUBSCRIBE,  (byte) 0x02);
        if (qos < 0)
            qos = 0;
        if (qos > 2)
            qos = 2;
        Topic = topic;
        packet.append((byte) (packetIdentifier >> 8));
        packet.append((byte) (packetIdentifier & 0xFF));
        packet.append(topic);
        packet.append((byte) qos);
    }

    /**
     * \brief Gibt an, ob nach dem Versand dieser Nachricht eine Antwort vom Broker
     * erwartet wird.
     */
    @Override
    public boolean mustBeConfirmed() {
        return true;
    }

    /**
     * \brief Die Nachricht wird zur wiederholten Sendung zurück geliefert.
     */
    @Override
    public MsgMqtt retryAction() throws MqttException {
        return this;
    }
}