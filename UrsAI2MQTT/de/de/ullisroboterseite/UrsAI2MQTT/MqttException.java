package de.ullisroboterseite.UrsAI2MQTT;;

/**
 * \bnief Exception für dieses Projekt.
 */
public class MqttException extends Exception {
    private static final long serialVersionUID = 1L;

    public final MqttErrorCode Reason;

    public MqttException(MqttErrorCode reasonCode) {
        Reason = reasonCode;
    }

    public MqttException(MqttErrorCode reasonCode, String message) {
        super(message);
        Reason = reasonCode;
    }

    public MqttException(MqttErrorCode reasonCode, String message, Throwable cause) {
        super(message, cause);
        Reason = reasonCode;
    }

    public MqttException(MqttErrorCode reasonCode, Throwable cause) {
        super(cause);
        Reason = reasonCode;
    }
}