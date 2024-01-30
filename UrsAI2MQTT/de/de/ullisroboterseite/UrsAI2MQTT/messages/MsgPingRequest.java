package de.ullisroboterseite.UrsAI2MQTT.messages;

import de.ullisroboterseite.UrsAI2MQTT.MqttErrorCode;
import de.ullisroboterseite.UrsAI2MQTT.MqttException;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ PINGREQ
 */
public class MsgPingRequest extends MsgMqtt {
    /**
     * Initializiert eine neue Instanz der MsgPingRequest-Klasse.
     */
    public MsgPingRequest() {
        super(MsgType.MQTTPINGREQ);
    }

    /**
     * \brief Erstellt eine neue Instanz der MsgPingRequest-Klasse anhand einen
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    MsgPingRequest(MqttPacketBuffer mpb) {
        super(mpb);
    }

    /**
     * \brief Die Aktion löst eine Exception aus.
     */
    @Override
    public MsgMqtt retryAction() throws MqttException {
        throw new MqttException(MqttErrorCode.ServerPingTimeout);
    }

    /**
     * \brief Gibt an, ob nach dem Versand dieser Nachricht eine Antwort vom Broker
     * erwartet wird.
     */
    @Override
    public boolean mustBeConfirmed() {
        return true;
    }
}