package de.ullisroboterseite.UrsAI2MQTT;;

public enum MqttErrorCode {
    NoError(0, ""), //                                  Alle Methoden
    InvalidState(-1,"Invalid state"), //                Alle Methoden
    IllegalArguments(-2, "Invalid endpoint"), //        connect
    SecurityProblem(-3, "Security violation"), //       connect
    ConnectionTimeout(-4, "Connection timeout"), //     connect
    IOError(-5, "IO-Fehler"), //                        connect
    StreamError(-6, "Socket getStream problem"), //     connect
    IllegalBlockingMode(-7,"Illegal blocking mode"), // connect
    ReadTimeout(-8,"Read timeout"), //                  Nachrichten einlesen
    InvalidPaketFormat(-9,"Invalid format"), //         Nachrichten einlesen
    XmitError(-10,"Xmit failure"), //                   Nachricht versenden
    ServerPingTimeout(-11, "Ping failure"), //
    EmptyTopic(-12, "Empty topic"), //                   Publish, Subscribe, Unsunscribe
    InvalidBinary(-13, "Invalid binary code"), //        Publish
    NotByteArray(-14, "Object is not a byte array") //   Publish
    ;

    public final String errorText;
    public final int errorCode;

    MqttErrorCode(int code, String text) {
        this.errorCode = code;
        errorText = text;
    }

    @Override
    public String toString() {
        return errorText;
    }
};