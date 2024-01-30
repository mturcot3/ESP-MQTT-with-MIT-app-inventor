package de.ullisroboterseite.UrsAI2MQTT.messages;

import de.ullisroboterseite.UrsAI2MQTT.MqttException;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ UNSUBSCRIBE
 */
public class MsgUnsubscribe extends MsgMqtt {
    public String Topic;
    /**
     * \brief Erstellt eine neue Instanz der MsgUnsubscribe-Klasse.
     * @param topic Der Wert für das Topic-Element.

     */
    public MsgUnsubscribe(String topic) {
        super(MsgType.MQTTUNSUBSCRIBE, (byte) 0x2);

        packet.append((byte) (packetIdentifier >> 8));
        packet.append((byte) (packetIdentifier & 0xFF));
        packet.append(topic);
        Topic = topic;
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
     * Die Nachricht wird zur wiederholten Sendung zurück geliefert.
     */
    @Override
    public MsgMqtt retryAction() throws MqttException {
        return this;
    }
}