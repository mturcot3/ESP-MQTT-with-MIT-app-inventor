package de.ullisroboterseite.UrsAI2MQTT;;

/**
 * \brief Zustände der Verbindung mit dem Broker.
 */
public enum MqttConnectionState {
    Disconnected, ///< \brief Der Client ist mit keinem Broker verbunden.
    Connecting, ///< \brief Der Client baut die Verbindung zum Broker auf.
    Connected, ///< \brief Der Client ist mit einem Broker verbunden.
    Disconnecting, ///< \brief Der Client baut die Verbindung mit dem Broker ab.
    ConnectionAbortet ///< \brief Die Verbindung konnte nicht aufgebaut werden bzw. wurde unterbrochen.
}