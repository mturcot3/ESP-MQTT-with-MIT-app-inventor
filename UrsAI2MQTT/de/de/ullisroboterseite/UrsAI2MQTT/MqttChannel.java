package de.ullisroboterseite.UrsAI2MQTT;;

/**
 * Schnittstelle zu einem MQTT-Übertragungskanal.
 */
public abstract class MqttChannel {
    /**
     * Stellt eine (Netzwerk-) Verbindung zum Broker her.
     *
     * @throws MqttException Die Verbindung besteht nicht mehr.
     */
    public abstract void connect() throws MqttException;

    /**
     *  Unterbricht die Verbindung.
     */
    public abstract void disconnect();

    /**
     * Gibt an, ob Daten zum Einlesen zur Verfügung stehen.
     *
     * @return true, wenn Daten eingelesen werden können.
     * @throws MqttException Die Verbindung besteht nicht mehr.
     */
    public abstract boolean available() throws MqttException;

    /**
     * Liefert ein einzelnes Byte.
     *
     * @return Das eingelesene Byte.
     * @throws MqttException IO-Fehler oder Timeout. Beides bedeutet
     *                       Verbindungsabbruch.
     */
    public abstract byte readByteTimeout() throws MqttException;

    /**
     * @brief Versendet ein Datenpaket (Byte-Array).
     * @param data Das zu versendende Byte-Array.
     * @throws MqttException Die Verbindung besteht nicht mehr.
     */
    public abstract void xmit(byte[] data) throws MqttException;
}