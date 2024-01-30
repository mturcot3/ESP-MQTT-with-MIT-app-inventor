package de.ullisroboterseite.UrsAI2MQTT.messages;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ PINGRESP
 */
public class MsgPingResponse extends MsgMqtt {
    /**
     * \brief Initializiert eine neue Instanz der MsgPingResponse-Klasse.
     */
    public MsgPingResponse() {
        super(MsgType.MQTTPINGRESP);
    }

    /**
     * \brief Erstellt eine neue Instanz der MsgPingResponse-Klasse anhand einen
     * empfagenen Datenpakets.
     *
     * @param mpb Das Datenpaket, aus dem die Nachricht extrahiert werden soll.
     * @throws MqttException Das Datenpaket ist nicht korrekt.
     */
    MsgPingResponse(MqttPacketBuffer mpb) {
        super(mpb);
    }
}