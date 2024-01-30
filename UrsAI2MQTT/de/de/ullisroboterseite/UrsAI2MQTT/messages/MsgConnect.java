package de.ullisroboterseite.UrsAI2MQTT.messages;

import de.ullisroboterseite.UrsAI2MQTT.MqttClient;
import de.ullisroboterseite.UrsAI2MQTT.MqttErrorCode;
import de.ullisroboterseite.UrsAI2MQTT.MqttException;

;

/**
 * \brief Repräsentiert eine MQTT-Nachricht vom Typ CONNECT.
 */
public class MsgConnect extends MsgMqtt {
    public static final Byte MQTT_VERSION = 4; // 3.1.1

    /**
     * \brief Initialisiert eine neue Instanz der MsgConnect-Klasse.
     *
     * @param cliendID     ClientID für diese Session.
     * @param cleanSession Clean-Session-Flag.
     * @param user         Username für die Anmeldung (kann null oder leer sein).
     * @param pass         Passwort
     * @param willTopic    Topic für 'last will'
     * @param willQos      Qos für last will'
     * @param willRetain   Retain-Flag für last will'
     * @param willMessage  für 'last will'
     * @param keepAlive    Sekunden für KeepAlive
     */
    public MsgConnect(String cliendID, boolean cleanSession, String user, String pass, String willTopic, byte willQos,
            boolean willRetain, String willMessage, int keepAlive) {
        this(cliendID, cleanSession, user, pass, willTopic, willQos, willRetain, getWillMessageBytes(willMessage),
                keepAlive);
    }

    private static byte[] getWillMessageBytes(String willMessage) {
        if (willMessage == null)
            return null;
        return willMessage.getBytes(MqttClient.Charset);
    }

    /**
     * \brief Initialisiert eine neue Instanz der MsgConnect-Klasse.
     *
     * @param cliendID     ClientID für diese Session.
     * @param cleanSession Clean-Session-Flag
     * @param user         Username für die Anmeldung (kann null oder leer sein).
     * @param pass         Passwort
     * @param willTopic    Topic für 'last will'
     * @param willQos      Qos für last will'
     * @param willRetain   Retain-Flag für last will'
     * @param willMessage  für 'last will'
     * @param keepAlive    Sekunden für KeepAlive
     */
    MsgConnect(String cliendID, boolean cleanSession, String user, String pass, String willTopic, byte willQos,
            boolean willRetain, byte[] willMessage, int keepAlive) {
        super(MsgType.MQTTCONNECT);

        if (willQos < 0)
            willQos = 0;
        if (willQos > 2)
            willQos = 2;
        if (user != null)
            if (user.isEmpty())
                user = null;
        if (willTopic != null)
            if (willTopic.isEmpty())
                willTopic = null;
        byte[] d = { 0x00, 0x04, 'M', 'Q', 'T', 'T', MQTT_VERSION };

        packet.append(d);

        byte willRetainByte = willRetain ? (byte) 1 : (byte) 0;
        byte v;

        if (willTopic != null) {
            v = (byte) (0x04 | (willQos << 3) | (willRetainByte << 5));
        } else {
            v = 0x00;
        }
        if (cleanSession) {
            v = (byte) (v | 0x02);
        }

        if (user != null) {
            v = (byte) (v | 0x80);

            if (pass != null) {
                v = (byte) (v | (0x80 >> 1));
            }
        }

        packet.append(v);
        packet.append((byte) (keepAlive >> 8));
        packet.append((byte) (keepAlive & 0xFF));
        packet.append(cliendID);

        if (willTopic != null) {
            packet.append(willTopic);
            packet.append((byte) (willMessage.length >> 8));
            packet.append((byte) (willMessage.length & 0xFF));
            packet.append(willMessage);
        }
        if (user != null) {
            packet.append(user);
            if (pass != null) {
                packet.append(pass);
            }
        }
    }

    /**
     * \brief Die Aktion löst eine Exception aus.
     */
    @Override
    public MsgMqtt retryAction() throws MqttException {
        throw new MqttException(MqttErrorCode.ConnectionTimeout);
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