package de.ullisroboterseite.UrsAI2MQTT;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;

/**
 * TCP-Client zur Kommunikation mit einem MQTT-Broker.
 */
public class MqttTcpChannel extends MqttChannel {
    Socket socket = null;
    OutputStream os;
    InputStream is;
    long socketTimeout; // Timeout für Socket-Operationen in ms
    int connectTimeout; // Timeout für den Verbindungsaufbau
    int keepAlive; // Timeout für Nachrichtenempfang und Ping
    String host;
    int port;

    /**
     * Initialisiert eine neue Instanz der Klasse \ref MqttNetworkClient.
     *
     * @param hostname             Hostname oder IP-Adresse des Brokers.
     * @param port                 Port für diesen Endpunkt.
     * @param ReadTimeoutSeconds   Timeout für Lese-Operationen.
     * @param connecTimeoutSeconds Timeout für den Verbindungsaufbau in Sekunden.
     * @param keepAliveSeconds     Timeout für den Nachrichtenempfang.
     */
    public MqttTcpChannel(String hostname, int portNumber, int ReadTimeoutSeconds, int connecTimeoutSeconds,
            int keepAliveSeconds) {
        socketTimeout = ReadTimeoutSeconds * 1000;
        connectTimeout = connecTimeoutSeconds * 1000;
        keepAlive = keepAliveSeconds;
        host = hostname;
        port = portNumber;
    }

    /**
     * \brief Stellt eine TCP-Verbindung zum eingestellten Broker her.
     *
     *
     * @throws MqttException Die Verbindung konnte nicht hergestellt werden.
     */
    @Override
    public void connect() throws MqttException {
        SocketAddress adr;

        // Endpunkt erstellen.
        try {
            adr = new InetSocketAddress(host, port);
        } catch (IllegalArgumentException e) {
            throw new MqttException(MqttErrorCode.IllegalArguments, e);
        } catch (SecurityException e) {
            throw new MqttException(MqttErrorCode.SecurityProblem, e);
        }

        // Socket einrichten und verbinden.
        socket = new Socket();
        try {
            socket.connect(adr, connectTimeout);
        } catch (SocketTimeoutException e) {
            throw new MqttException(MqttErrorCode.ConnectionTimeout, e);
        } catch (IllegalArgumentException e) {
            throw new MqttException(MqttErrorCode.IllegalArguments, e);
        } catch (java.nio.channels.IllegalBlockingModeException e) {
            throw new MqttException(MqttErrorCode.IllegalBlockingMode, e);
        } catch (IOException e) {
            throw new MqttException(MqttErrorCode.IOError, e);
        }

        // Input- und Output-Streams anlegen.
        try {
            os = socket.getOutputStream();
            is = socket.getInputStream();
        } catch (IOException e) {
            throw new MqttException(MqttErrorCode.StreamError, e); // Wirft Exception
        }
    }

    /**
     * Schließt den Socket.
     */
    @Override
    public void disconnect() {
        try {
            os.close();
            os = null;
        } catch (Exception e) {
            // Nothing to do
        }
        try {
            is.close();
            is = null;
        } catch (Exception e) {
            // Nothing to do
        }
        try {
            socket.close();
            socket = null;
        } catch (Exception e) {
            // Nothing to do
        }
    }

    /**
     * Gibt an, ob Daten zum Einlesen zur Verfügung stehen.
     *
     * @return true, wenn Daten eingelesen werden können.
     * @throws MqttException Die Verbindung besteht nicht mehr.
     */
    @Override
    public boolean available() throws MqttException {
        try {
            return is.available() > 0;
        } catch (Exception e) {
            throw new MqttException(MqttErrorCode.IOError, e);
        }
    }

    /**
     * Liest ein Byte aus dem Input-Stream.
     *
     * @return Das eingelesene Byte.
     * @throws MqttException Die Verbindung besteht nicht mehr.
     */
    @Override
    public byte readByteTimeout() throws MqttException {
        long previousMillis = System.currentTimeMillis();
        while (!available()) {
            long currentMillis = System.currentTimeMillis();
            if (currentMillis - previousMillis >= socketTimeout)
                throw new MqttException(MqttErrorCode.ReadTimeout);

            Thread.yield();
        }
        try {
            return (byte) is.read();
        } catch (IOException e) {
            throw new MqttException(MqttErrorCode.IOError, e);
        }
    }

    /**
     * @brief Versendet ein Datenpaket (Byte-Array).
     * @param data Das zu versendende Byte-Array.
     * @throws MqttException Die Verbindung besteht nicht mehr.
     */
    @Override
    public void xmit(byte[] data) throws MqttException {
        try {
            os.write(data, 0, data.length);
        } catch (Exception e) {
            throw new MqttException(MqttErrorCode.XmitError, e);
        }
    }

}