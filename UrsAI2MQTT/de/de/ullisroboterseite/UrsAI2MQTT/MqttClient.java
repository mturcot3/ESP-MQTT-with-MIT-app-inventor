package de.ullisroboterseite.UrsAI2MQTT;

import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import android.util.*;
import de.ullisroboterseite.UrsAI2MQTT.messages.*;

/**
 * MQTT-Client
 */
public class MqttClient { // implements IMqttListener {
    // Netzwerkzugriffe müssen in separatem Thread erfolgen
    private static class Runner {
        static void start(Runnable r) {
            Thread t1 = new Thread(r);
            t1.start();
        }
    }

    static final String LOG_TAG = UrsAI2MQTT.LOG_TAG;

    public static final Charset Charset = StandardCharsets.UTF_8;
    volatile MqttConnectionState connectionState = MqttConnectionState.Disconnected;
    UrsAI2MQTT _appListener = null;

    MqttMessageHandler mrt = null;

    /**
     * Initialisiert eine neue Instanz der Klasse UrsMqttClient.
     *
     * @param listener Listener für MQTT-Nachrichten.
     */
    public MqttClient(UrsAI2MQTT listener) {
        _appListener = listener;
    }

    /**
     * Event wird ausgelöst, wenn sich der Verbindungszustand geändert hat.
     *
     * @param newState  Der neue Verbindungszustand
     * @param errorCode
     */
    public void ConnectionStateChangedEvent(MqttConnectionState newState, int errorCode, String errorText) {
        connectionState = newState;
        String logMsg = "Verbindungsstatus ist: " + newState.toString();
        if (errorText != null)
            if (!errorText.isEmpty())
                logMsg += " (" + errorText + ")";
        Log.d(LOG_TAG, logMsg);
        _appListener.ConnectionStateChangeCallback(newState, errorCode, errorText);
    }

    /**
     *  Methode wird beim Empfang von einer PUBLISH-Nachricht aufgerufen.
     */
    public void PublishedReceivedCallback(MsgPublish mp) {
        _appListener.PublishedReceivedCallback(mp);
    }

    /**
     * Methode wird beim Empfang von einer SUBACK-Nachricht aufgerufen.
     */
    public void SuBackReceivedCallback(MsgSubAck msg) {
        _appListener.SuBackReceivedCallback(msg.SubscribeFailure, msg.MaxQoS, msg.Topic);
    }

    /**
     * Methode wird beim Empfang von einer UNSUBACK-Nachricht aufgerufen.
      */
    public void UnSuBackReceivedCallback(MsgUnSubAck msg) {
        _appListener.UnSuBackReceivedCallback(msg.Topic);
    }

    /**
     * Konvertiert ein Byte Array in eine ASCII-HEX-Darstellung
     */
    public static String byteArrayToHex(byte[] a) {
        StringBuilder sb = new StringBuilder(a.length * 2);
        for (byte b : a)
            sb.append(String.format("%02x ", b));
        return sb.toString();
    }

    /**
     * Baut eine Verbindung zum Broker auf.
     *
     * @param broker       Hostname für den Broker
     * @param port         Port für die Verbindung
     * @param id           Eindeutige Client-ID
     * @param user         Username
     * @param pass         Passwort
     * @param willTopic    Topic für 'last will'
     * @param willQos      QoS für 'lat will'
     * @param willRetain   Retain-Flag für 'last will'
     * @param willMessage  'last will'-Nachricht
     * @param cleanSession 'Clean Session'-Flag
     * @param keepAlive    Sekunden für KeepAlive
     * @param IoTimeout    Timeout für den Datentransfer in Sekunden
     */
    public void connect(final String broker, final int port, final String id, final boolean cleanSession,
            final String user, final String pass, final String willTopic, final byte willQos, final boolean willRetain,
            final String willMessage, final int keepAlive, final int IoTimeout) {

        ConnectionStateChangedEvent(MqttConnectionState.Connecting, 0, "");

        final MqttClient mqttClient = this; // wird für die Lambda-Funktion benötigt

        MqttChannel mqttChannel = new MqttTcpChannel(broker, port, IoTimeout, IoTimeout, keepAlive);

        MsgConnect mc = new MsgConnect(id, cleanSession, user, pass, willTopic, willQos, willRetain, willMessage,
                keepAlive);
        mrt = new MqttMessageHandler(mqttChannel, mqttClient, keepAlive, mc);
        mrt.start();
    }

    /**
     * Trennt die Verbindung um Broker.
     */
    public void disconnect() {
        ConnectionStateChangedEvent(MqttConnectionState.Disconnecting, 0, "");
        Runner.start(new Runnable() {
            public void run() {
                mrt.xmit(new MsgDisconnect());
                mrt.stopRequest = true;
                mrt = null;
            } // run
        }); // Runnable
    }

    /**
     * Versendet eine SUBSCRIBE-Nachricht.
     *
     * @param topic Topic, dass abboniert werden soll.
     * @param qos   QoS für dieses Topic.
     */
    void subscribe(final String topic, final byte qos) {
        Runner.start(new Runnable() {
            public void run() {
                mrt.xmit(new MsgSubscribe(topic, qos));
            } // run
        }); // Runnable
    }

    /**
     * Versendet eine UNSUBSCRIBE-Nachricht.
     *
     * @param topic Topic, dass nicht mehr abboniert sein soll.
     */
    void unsubscribe(final String topic) {
        Runner.start(new Runnable() {
            public void run() {
                mrt.xmit(new MsgUnsubscribe(topic));
            } // run
        }); // Runnable
    }

    /**
     * Versendet eine PUBLISH-Nachricht.
     *
     * @param topic    Topic für diese Nachricht
     * @param message  Nutzdaten.
     * @param retained Retained-Flag.
     */
    public void publish(String topic, String message, boolean retained, byte qos) {
        publish(topic, message.getBytes(MqttClient.Charset), retained, qos);
    }

    /**
     * Versendet eine PUBLISH-Nachricht.
     *
     * @param topic    Topic für diese Nachricht
     * @param message  Nutzdaten.
     * @param retained Retained-Flag.
     * @param qos      QoS.
     */
    public void publish(final String topic, final byte[] payload, final boolean retained, final byte qos) {
        Runner.start(new Runnable() {
            public void run() {
                mrt.xmit(new MsgPublish(topic, payload, retained, qos));
            } // run
        }); // Runnable
    }
}