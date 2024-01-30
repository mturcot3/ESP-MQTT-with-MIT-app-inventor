package de.ullisroboterseite.UrsAI2MQTT;

import java.util.Iterator;

import android.util.*;
import de.ullisroboterseite.UrsAI2MQTT.messages.*;

/**
 * \brief Thread, der Nachrichten des Brokers einliest und analysiert.
 */
public class MqttMessageHandler extends Thread {
    static final String LOG_TAG = UrsAI2MQTT.LOG_TAG;
    MqttChannel mqttChannel;
    MqttClient mqttListener;
    long keepAliveMilliseconds;
    MsgConnect mc = null;

    long nextPing = 0; // Zeitpunkt zu dem der nächste PinRequest abgesetzt werden soll.

    boolean isConnected = false; // kein PingRequest, wenn nicht ordenrlich verbunden.

    // Queue für ausgehende Nachrichten, die vom Broker quittiert werden müssen.
    MqttMessageQueue OutboundQueue = new MqttMessageQueue("OutboundQueue");

    // Queue für vom Broker kommende Nachrichten, für die der Broker eine Quitting erwartet.
    // und nicht wiederholt zurück gemeldet werden sollen (PUBLISH (received))
    MqttMessageQueue InboundQueue = new MqttMessageQueue("InboundQueue");

    boolean stopRequest = false; // true, wenn der Thread beendet werden soll.

    MqttException abortException = null; // Exception, die zum Abbruch der Empfangsschleife geführt hat.

    /**
     * \brief Initialisiert eine neue Instanz der Klasse \ref MqttReceiverThread.
     * Bei jedem Verbindungsaufbau wird eine neue Instanz dieser Klasse erzeugt und
     * beim Verbindungsabbau wieder zerstört. Die erste Aktion der übergeordneten
     * Schicht ist das Versenden eine CONNECT-Nachricht.
     *
     * @param channel          Instanz der Klasse \ref MqttNetworkClient, die zur
     *                         Ein- und Ausagbe benutzt werden soll.
     * @param listener         Instanz des Interface \ref IMqttListener, das
     *                         Nachrichten verarbeiten soll.
     * @param keepAliveSeconds Anzahl Sekunden für Timeout
     */
    public MqttMessageHandler(MqttChannel channel, MqttClient listener, int keepAliveSeconds, MsgConnect msgConnect) {
        mqttChannel = channel;
        mqttListener = listener;
        keepAliveMilliseconds = keepAliveSeconds * 1000;
        mc = msgConnect;
    }

    // Der Thread wird beendet bei
    // - stopRequest
    // - Fehler beim TCP-Zugriff
    // - ToDo: negatives CONNACK
    // - ToDo: TimeOut bei erwarteter Antwort vom Broker
    @Override
    public void run() {
        Log.v(LOG_TAG, "Thread gestartet");

        try {
            // Verbindung herstellen
            mqttChannel.connect();
            xmit(mc);

            // Nachrichtenschleife
            while (!stopRequest) {
                handleIncommingMessage();
                if (!stopRequest)
                    handlePendingActions();
                Thread.yield();
            }
        } catch (MqttException ex) {
            abortException = ex;
        }

        mqttChannel.disconnect();

        if (abortException != null) // ToDo: Ist dies die richtige Vorgehensweise?
            mqttListener.ConnectionStateChangedEvent(MqttConnectionState.ConnectionAbortet,
                    abortException.Reason.errorCode, abortException.getMessage());
        else
            mqttListener.ConnectionStateChangedEvent(MqttConnectionState.Disconnected, 0, "");

        String lMsg = "Thread beendet";
        if (abortException != null)
            lMsg += " " + abortException.toString();
        Log.v(LOG_TAG, lMsg);
    }

    /**
     * Prüft auf ausstehende Aktionen und behandelt diese.
     *
     * @throws MqttException Austehende Aktionen wurden nicht oder nicht zeitgerecht
     *                       behandelt.
     */
    private void handlePendingActions() throws MqttException {
        long t = System.currentTimeMillis();
        long timetouted = t - keepAliveMilliseconds;

        try {
            OutboundQueue.lock();
            Iterator<MsgMqtt> iterator = OutboundQueue.iterator();
            while (iterator.hasNext()) {
                MsgMqtt msg = iterator.next();
                if (msg.messageSentAt < timetouted) {
                    Log.d(LOG_TAG, "Timeout: " + msg.toString());
                    MsgMqtt fmsg = msg.retryAction(); // Exception, falls keine Wiederholung vorgesehen
                    if (fmsg != null)
                        xmit(fmsg, true); // Exception, falls Verbindung getrennt
                }
            }
        } finally {
            OutboundQueue.unlock();
        }

        if (isConnected && t > nextPing) { // CONNACK setzt isConnected und nextPing
            MsgPingRequest mp = new MsgPingRequest();
            Log.d(LOG_TAG, "ping 0");
            xmit(mp);
            nextPing = System.currentTimeMillis() + keepAliveMilliseconds;
        }
    }

    /**
     * Liest eingehende Nachrichten ein und reagiert darauf.
     *
     * @throws MqttException Die Nachricht konnte nicht korrekt eingelesen oder
     *                       entschlüsselt werden.
     */
    private void handleIncommingMessage() throws MqttException {
        if (!mqttChannel.available()) // Es stehen keine Daten an
            return;

        MsgMqtt msg = MsgMqtt.fromBuffer(MqttPacketBuffer.fromStream(mqttChannel));
        if (msg == null) {
            return;
        }

        Log.d(LOG_TAG, "Message  got: " + msg.toString());

        nextPing = System.currentTimeMillis() + keepAliveMilliseconds;

        switch (msg.getType()) {
            case MQTTPUBLISH:
                MsgPublish mpu = (MsgPublish) msg;
                switch (mpu.getQoS()) {
                    case 0:
                        mqttListener.PublishedReceivedCallback(mpu);
                        break;
                    case 1:
                        mqttListener.PublishedReceivedCallback(mpu);
                        xmit(new MsgPubAck(mpu.packetIdentifier));
                        break;
                    case 2:
                        if (!InboundQueue.find(mpu.packetIdentifier)) // nur einmal zurück melden
                            mqttListener.PublishedReceivedCallback(mpu);
                        xmit(new MsgPubRec(mpu.packetIdentifier));
                        break;
                } // switch (mpu.getQoS())
                break;
            case MQTTPUBACK:
                OutboundQueue.removeSync(MsgType.MQTTPUBLISH, msg.packetIdentifier);
                break;
            case MQTTPUBREC:
                OutboundQueue.removeSync(MsgType.MQTTPUBLISH, msg.packetIdentifier);
                xmit(new MsgPubRel(msg.packetIdentifier));
                break;
            case MQTTPUBREL:
                InboundQueue.removeSync(MsgType.MQTTPUBLISH, msg.packetIdentifier);
                OutboundQueue.removeSync(MsgType.MQTTPUBREC, msg.packetIdentifier);
                xmit(new MsgPubComp(msg.packetIdentifier));
                break;
            case MQTTPUBCOMP:
                OutboundQueue.removeSync(MsgType.MQTTPUBREL, msg.packetIdentifier);
                break;
            case MQTTPINGREQ:
                MsgPingResponse mp = new MsgPingResponse();
                xmit(mp);
                break;
            case MQTTPINGRESP:
                OutboundQueue.remove(MsgType.MQTTPINGREQ);
                break;
            case MQTTCONNACK:
                OutboundQueue.remove(MsgType.MQTTCONNECT);
                checkConnectedAck(((MsgConnAck) msg)); // bricht ggf. mit Exception ab
                isConnected = true;
                mqttListener.ConnectionStateChangedEvent(MqttConnectionState.Connected, 0, "");
                Log.d(LOG_TAG, "ConnectedAckCallback Connect ok");
                break;
            case MQTTSUBACK:
                MsgSubscribe m = (MsgSubscribe) OutboundQueue.removeSync(MsgType.MQTTSUBSCRIBE, msg.packetIdentifier);
                ((MsgSubAck) msg).Topic = m.Topic; // SUBACK hat kein Topic
                mqttListener.SuBackReceivedCallback((MsgSubAck) msg);
                break;
            case MQTTUNSUBACK:
                MsgUnsubscribe mu = (MsgUnsubscribe) OutboundQueue.removeSync(MsgType.MQTTUNSUBSCRIBE,
                        msg.packetIdentifier);
                ((MsgUnSubAck) msg).Topic = mu.Topic; // UNSUBACK hat kein Topic
                mqttListener.UnSuBackReceivedCallback((MsgUnSubAck) msg);
                break;
            default:
                break;
        }
    }

    // Prüft die CONNACK-Meldung
    // Wirft eine Exception, wenn die Verbindung abgewiesen wurde
    void checkConnectedAck(MsgConnAck msg) throws MqttException {
        if (msg.connectReturnCode != 0) {
            String text = "";
            switch (msg.connectReturnCode) {
                case 1:
                    text = "Unacceptable protocol version";
                    break;
                case 2:
                    text = "Identifier rejected";
                    break;
                case 3:
                    text = "Server unavailable";
                    break;
                case 4:
                    text = "Bad user name or password";
                    break;
                case 5:
                    text = "Not authorized";
                    break;
                default:
                    break;
            }
            Log.d(LOG_TAG, "ConnectedAckCallback Connect Fehler");
            throw new MqttException(MqttErrorCode.SecurityProblem, text);
        }
    }

    /**
     * Fügt die Nachricht der Liste der Nachrichten hinzu, die auf eine
     * Antwort des Brokers warten.
     * Dies wird durch mustBeConfirmed der Nachricht bestimmt
     *
     * @param msg Die anzufügende Nachricht.
     */
    void addPendingMessage(MsgMqtt msg) {

        OutboundQueue.addSync(msg);

        // bei PUBLISH && QoS == 2 besteht die Möglichkeit, dass die Nachricht
        // mehrfach vom Server quittiert wird.
        if (msg.getType() == MsgType.MQTTPUBLISH) {
            MsgPublish mp = (MsgPublish) msg;
            if (mp.getQoS() == 2) {
                if (!InboundQueue.find(mp.packetIdentifier))
                    InboundQueue.addSync(mp);
            }
        }
    }

    /**
     * Versendet die angegebene MQTT-Nachricht.
     *
     * @param msg Die MQTT-Nachricht, die versendet werden soll.
     * @throws MqttException Die Nachricht konnte nicht versandt werden.
     */
    public void xmit(MsgMqtt msg)  {
        xmit(msg, false);
    }

    // intener Aufruf ohne Nachrichtenspeicher bei Wiederholung
    private void xmit(MsgMqtt msg, boolean isRetry)  {
        byte[] buf = msg.getRawBuffer();

        Log.d(LOG_TAG, "Message sent: " + msg.toString() + " Retry: " + !isRetry);

        try {
            mqttChannel.xmit(buf);
        } catch (Exception e) { // führt zum Abbruch
            abortException= new MqttException(MqttErrorCode.XmitError,
            MqttErrorCode.XmitError.errorText + ": " + msg.getTypeName(), e);
            stopRequest = true;
            return;
        }

        msg.messageSentAt = System.currentTimeMillis();

        if (!isRetry)
            if (msg.mustBeConfirmed()) // Die Nachricht bestimmt, ob eine Wiederholung notwendig ist.
                addPendingMessage(msg);
    }
}