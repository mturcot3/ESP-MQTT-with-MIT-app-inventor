package de.ullisroboterseite.UrsAI2MQTT;

// Autor: https://UllisRoboterSeite.de

// Doku:  https://UllisRoboterSeite.de/android-AI2-MQTT.html
// Created: 2019-10-14
//
// Version 1 (2019-10-14)
// -------------------------
// - Basis-Version
//
// Version 1.1 (2019-11-09)
// -------------------------
// - Falsche Packet-ID bei Publish und QoS=1
//
// Version 1.2 (2019-12-04)
// -------------------------
// - Benutzername wurde nicht weiter gegeben. Dies hat die Anmeldung bei Brokern verhindert, die eine Autorisierung verlangen.
//
// Version 1.3 (2019-12-04)
// -------------------------
// - Bei ungültiger Autorisierung wurde die Verbindung nicht unterbrochen.
//
// Version 1.4 (2019-12-05)
// -------------------------
// - Packet-IDs > 127 wurden falsch aufgelöst.
//
// Version 1.5 (2020-01-22)
// -------------------------
// - Das Senden von Ping-Requests überarbeitet.
//
// Version 1.6 (2020-01-30)
// -------------------------
// - PublishByteArray & PublishedByteArrayReceived hinzugefügt.
//
// Version 1.7 (2020-03-06)
// -------------------------
// - PublishByteArray fängt null-Pointer ab.
//- IsNull hinzugefügt.
//
// Version 1.8 (2020-03-20)
// -------------------------
// - Speicher für unbehandelte Nachrichten wird beim Disconnect gelöscht.
// - Auf Beendigung des MessageHandler-Threads wird beim Disconnect gewartet,
//   bevor die TCP-Verbindung getrennt wird.
// - Bei Übertragungsfehlern (Xmit) enthält LastErrorMessage die Nachrichtenart
//
// Version 1.9 (2020-10-14)
// -------------------------
// - Fehlerhafte Stringvergleiche behoben [String == "" -> String.isEmpty()]
// - Internen Abläufe überarbeitet.

import java.util.*;

import com.google.appinventor.components.annotations.*;
import com.google.appinventor.components.common.*;
import com.google.appinventor.components.runtime.*;
import android.os.Handler;
import de.ullisroboterseite.UrsAI2MQTT.messages.MsgPublish;

@DesignerComponent(version = 1, //
        versionName = "1.9", //
        dateBuilt = "2020-10-14", //
        description = "AI2 extension block for MQTT communication.", //
        category = com.google.appinventor.components.common.ComponentCategory.EXTENSION, //
        nonVisible = true, //
        helpUrl = "http://UllisRoboterSeite.de/android-AI2-MQTT-de.html", //
        iconName = "aiwebres/mqtt.png")
@SimpleObject(external = true)
@UsesPermissions(permissionNames = "android.permission.INTERNET,android.permission.WAKE_LOCK,android.permission.ACCESS_NETWORK_STATE")
public class UrsAI2MQTT extends AndroidNonvisibleComponent { //implements IExtensionListener {
    static final String LOG_TAG = "MQTT";
    final Handler handler = new Handler();

    private MqttClient _client = new MqttClient(this);
    private ArrayList<String> byteArraySubscriptionList = new ArrayList<String>();
    private ArrayList<String> pendingByteArraySubscriptionList = new ArrayList<String>();

    public UrsAI2MQTT(ComponentContainer container) {
        super(container.$form());
    } // ctor

    // #region Properties
    // ---- Broker --------------------------------------------------------
    protected volatile String _broker = "";

    @SimpleProperty(description = "The IP address or hostname of the server to connect to.")
    public String Broker() {
        return _broker;
    }

    @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_STRING, defaultValue = "")
    @SimpleProperty(description = "The IP address or hostname of the server to connect to.")
    public void Broker(String value) {
        _broker = value;
    }

    // ---- Port --------------------------------------------------------
    protected volatile int _port = 1883;

    @SimpleProperty(description = "The port number of the server to connect to.")
    public int Port() {
        return _port;
    }

    @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_NON_NEGATIVE_INTEGER, defaultValue = "1883")
    @SimpleProperty(description = "The port number of the server to connect to.")
    public void Port(int value) {
        if (value > 0)
            _port = value;
    }

    // --- IO Timeout -----------------------------------------
    protected volatile int _ioTimeout = 5;

    @SimpleProperty(description = "Timeout for data transfer in seconds.")
    public int IoTimeout() {
        return _ioTimeout;
    }

    @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_NON_NEGATIVE_INTEGER, defaultValue = "5")
    @SimpleProperty(description = "Timeout for data transfer in seconds.")
    public void IoTimeout(int value) {
        if (value >= 0)
            _ioTimeout = value;
    }

    // --- Keep alive -----------------------------------------------
    protected volatile int _keepAlive = 5;

    @SimpleProperty(description = "Keep alive time in seconds.")
    public int KeepAlive() {
        return _keepAlive;
    }

    @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_NON_NEGATIVE_INTEGER, defaultValue = "5")
    @SimpleProperty(description = "Keep alive time in seconds.")
    public void KeepAlive(int value) {
        if (value > 0)
            _keepAlive = value;
    }

    // ---- ClientID ------------------------------------------------
    protected volatile String _clientID = "";

    @SimpleProperty(description = "The unique client Id. If this field is blank a random GUID is used.")
    public String ClientID() {
        return _clientID;
    }

    @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_STRING, defaultValue = "")
    @SimpleProperty(description = "The unique client Id. If this field is blank, a random GUID is used.")
    public void ClientID(String value) {
        _clientID = value;
    }

    // ---- UserName ------------------------------------------------
    protected volatile String _user = "";

    @SimpleProperty(description = "The user name used authentication and authorization.")
    public String UserName() {
        return _user;
    }

    @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_STRING, defaultValue = "")
    @SimpleProperty(description = "The user name used authentication and authorization.")
    public void UserName(String value) {
        _user = value;
    }

    // ---- Password ------------------------------------------------
    protected volatile String _password = "";

    @SimpleProperty(description = "The password used authentication and authorization.")
    public String Password() {
        return _password;
    }

    @DesignerProperty(editorType = PropertyTypeConstants.PROPERTY_TYPE_STRING, defaultValue = "")
    @SimpleProperty(description = "The password used authentication and authorization.")
    public void Password(String value) {
        _password = value;
    }

    // ---- IsConnected ----------------------------------------------
    @SimpleProperty(description = "true: Client is connected to a MQTT broker.")
    public boolean IsConnected() {
        return _client.connectionState == MqttConnectionState.Connected;
    }

    // ---- IsDisConnected ----------------------------------------------
    @SimpleProperty(description = "true: Client is disconnected from the MQTT broker.")
    public boolean IsDisconnected() {
        return (_client.connectionState == MqttConnectionState.Disconnected)
                || (_client.connectionState == MqttConnectionState.ConnectionAbortet);
    }

    // ---- ConnectionState ----------------------------------------------

    @SimpleProperty(description = "The connection state:\n" //
            + "0: Disconnected. The client is not connected to a broker.\n" //
            + "1: Connecting. The client is currently creating a connection to a MQTT broker.\n" //
            + "2: Connected. The client is connected to a MQTT broker.\n" //
            + "3: Disconnecting. The client is currently disconnecting from the MQTT broker.\n" //
            + "4: ConnectionAbortet. The connection could not be established or was interrupted.")
    public int ConnectionState() {
        return _client.connectionState.ordinal();
    }

    // ---- LastError ------------------------------------------------
    protected volatile String _lastErrMsg = ""; // Text des letzten Fehlers

    @SimpleProperty(description = "Returns a text message about the last error.")
    public String LastErrorMessage() {
        return _lastErrMsg;
    }

    protected volatile int _lastErrorCode = 0; // Fehlercode des letzten Fehlers

    @SimpleProperty(description = "Returns the code of the last error.")
    public int LastErrorCode() {
        return _lastErrorCode;
    }

    protected volatile String _lastAction = ""; // Fehlercode des letzten Fehlers

    @SimpleProperty(description = "Returns the last Action the error code belongs to.")
    public String LastAction() {
        return _lastAction;
    }

    private void setErrorInfo(MqttErrorCode ec) {
        _lastAction = new Throwable().getStackTrace()[1].getMethodName();
        _lastErrorCode = ec.ordinal();
        _lastErrMsg = ec.toString();
    }
    // #endregion Properties

    // #region Methods
    // #region Connect
    @SimpleFunction(description = "Connects to a MQTT broker. ConnectionState changes to 1: 'connecting'.")
    public void ConnectWithLastWill(boolean CleanSession, String WillTopic, String WillMessage, boolean WillRetain,
            byte WillQoS) {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsDisconnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        if (WillTopic == null) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }

        if (_clientID.isEmpty())
            _clientID = UUID.randomUUID().toString();

        _client.connect(_broker, _port, _clientID, CleanSession, _user, _password, WillTopic, WillQoS, WillRetain,
                WillMessage, _keepAlive, _ioTimeout);
    }

    @SimpleFunction(description = "Connects to a MQTT broker. ConnectionState changes to 1: 'connecting'.")
    public void Connect(boolean CleanSession) {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsDisconnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }

        if (_clientID.isEmpty())
            _clientID = UUID.randomUUID().toString();

        _client.connect(_broker, _port, _clientID, CleanSession, _user, _password, null, (byte) 0, false, null,
                _keepAlive, _ioTimeout);
    }
    // #endregion

    // #region Subscribe
    @SimpleFunction(description = "Subscribes a topic.")
    public void Subscribe(String Topic, byte QoS) {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsConnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        if (Topic == null) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (Topic.isEmpty()) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        _client.subscribe(Topic, QoS);
    }

    @SimpleFunction(description = "Subscribes a topic.")
    public void SubscribeByteArray(String Topic, byte QoS) {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsConnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        if (Topic == null) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (Topic.isEmpty()) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (!pendingByteArraySubscriptionList.contains(Topic))
            pendingByteArraySubscriptionList.add(Topic);
        _client.subscribe(Topic, QoS);
    }

    @SimpleFunction(description = "Unsubscribes a topic.")
    public void Unsubscribe(String Topic) {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsConnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        if (Topic == null) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (Topic.isEmpty()) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        pendingByteArraySubscriptionList.remove(Topic);
        _client.unsubscribe(Topic);
    }
    // #endregion

    // #region Publish
    @SimpleFunction(description = "Publishes a MQTT message.")
    public void PublishEx(String Topic, String Message, boolean RetainFlag, int QoS) {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsConnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        if (Topic == null) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (Topic.isEmpty()) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        _client.publish(Topic, Message, RetainFlag, (byte) QoS);
    }

    @SimpleFunction(description = "Publishes a MQTT message. Retain flag is false, QoS is 0.")
    public void Publish(String Topic, String Message) {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsConnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        if (Topic == null) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (Topic.isEmpty()) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        _client.publish(Topic, Message, false, (byte) 0);
    }

    // ----- Binary Message
    byte[] StringToBytes(String inp) throws NumberFormatException {
        inp = inp.replace(',', ';');
        String[] bs = inp.split(";");
        byte[] bytes = new byte[bs.length];

        for (int i = 0; i < bs.length; i++) {
            int b = Integer.decode(bs[i].trim());
            if (b > 255 || b < 0)
                throw new NumberFormatException();
            bytes[i] = (byte) b;
        }
        return bytes;
    }

    @SimpleFunction(description = "Publishes a binary coded MQTT message.")
    public void PublishBinary(String Topic, String BinaryMessage, boolean RetainFlag, int QoS) {
        byte[] bytes;

        setErrorInfo(MqttErrorCode.NoError);
        if (!IsConnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        if (Topic == null) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (Topic.isEmpty()) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }

        try {
            bytes = StringToBytes(BinaryMessage);
        } catch (Exception e) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        _client.publish(Topic, bytes, false, (byte) 0);
    }

    @SimpleFunction(description = "Publishes a binary array.")
    public void PublishByteArray(String Topic, Object ByteArray, boolean RetainFlag, int QoS) {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsConnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }
        if (Topic == null) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (Topic.isEmpty()) {
            setErrorInfo(MqttErrorCode.EmptyTopic);
            return;
        }
        if (ByteArray == null) {
            setErrorInfo(MqttErrorCode.NotByteArray);
            return;
        }
        if (!ByteArray.getClass().equals(byte[].class)) {
            setErrorInfo(MqttErrorCode.NotByteArray);
            return;
        }
        _client.publish(Topic, (byte[]) ByteArray, RetainFlag, (byte) QoS);
    }

    @SimpleFunction(description = "Test whether an object is null.")
    public boolean IsNull(Object Object) {
        return Object == null;
    }
    // #endregion Publish

    // #region Disconnect

    @SimpleFunction(description = "Disconnects from the broker.")
    public void Disconnect() {
        setErrorInfo(MqttErrorCode.NoError);
        if (!IsConnected()) {
            setErrorInfo(MqttErrorCode.InvalidState);
            return;
        }

        _client.disconnect();
    }
    // #endregion Disconnect

    // #endregion Methods

    // #region Events
    @SimpleEvent(description = "Connection state has changed.")
    public void ConnectionStateChanged(int NewState, String StateString) {
        EventDispatcher.dispatchEvent(this, "ConnectionStateChanged", NewState, StateString);
    }

    @SimpleEvent(description = "Message received.")
    public void PublishedReceived(String Topic, String Payload, String Message, boolean RetainFlag, boolean DupFlag) {
        EventDispatcher.dispatchEvent(this, "PublishedReceived", Topic, Payload, Message, RetainFlag, DupFlag);
    }

    @SimpleEvent(description = "Message with byte array received.")
    public void PublishedByteArrayReceived(String Topic, Object ByteArray, boolean RetainFlag, boolean DupFlag) {
        EventDispatcher.dispatchEvent(this, "PublishedByteArrayReceived", Topic, ByteArray, RetainFlag, DupFlag);
    }

    @SimpleEvent(description = "SUBACK Message received.")
    public void SuBackReceived(boolean Failure, int MaxQos, String Topic) {
        EventDispatcher.dispatchEvent(this, "SuBackReceived", Failure, MaxQos, Topic);
    }
    // #endregion

    // #region IExtensionListener
 //   @Override
    public void ConnectionStateChangeCallback(final MqttConnectionState newState, int errorCode, String errorText) {
        _lastErrorCode = errorCode;
        _lastErrMsg = errorText;
        handler.post(new Runnable() {
            public void run() {
                ConnectionStateChanged(newState.ordinal(), newState.toString());
            } // run
        }); // post
    }

 //   @Override
    public void PublishedReceivedCallback(MsgPublish mp) {
        // public void PublishedReceivedCallback(final String topic, byte[] payload,
        // final String msg, final boolean retain,
        // final boolean dup) {
        final String topic;
        final byte[] payload;
        final String msg;
        final boolean retain;
        final boolean dup;
        String result;

        topic = mp.getTopic();
        payload = mp.getPayload();
        retain = mp.getRetainFlag();
        dup = mp.getDupFlag();

        // Prüfen, ob Byte Array
        for (String tp : byteArraySubscriptionList) {
            if (TopicMatcher.topicMatchesSubscription(tp, topic) == TopicMatcher.MOSQ_MATCH) {
                // Ist Byte Array
                handler.post(new Runnable() {
                    public void run() {
                        PublishedByteArrayReceived(topic, payload, retain, dup);
                    } // run
                }); // post
                return;
            }
        }

        // 'Normale'-String-Nachricht
        msg = mp.getPayloadString();
        result = "";
        for (int i = 0; i < payload.length; i++) {
            int temp = payload[i];
            if (temp < 0)
                temp += 256;
            result += ";" + temp;
        }
        if (result.length() > 0)
            result = result.substring(1);

        final String pl = result;
        handler.post(new Runnable() {
            public void run() {
                PublishedReceived(topic, pl, msg, retain, dup);
            } // run
        }); // post
    }

//    @Override
    public void SuBackReceivedCallback(final boolean failure, final int maxQos, final String topic) {
        if (failure)
            pendingByteArraySubscriptionList.remove(topic);
        else {
            if (pendingByteArraySubscriptionList.contains(topic)) {
                pendingByteArraySubscriptionList.remove(topic);
                byteArraySubscriptionList.add(topic);
            }
        }

        handler.post(new Runnable() {
            public void run() {
                SuBackReceived(failure, maxQos, topic);
            } // run
        }); // post
    }

 //   @Override
    public void UnSuBackReceivedCallback(String topic) {
        pendingByteArraySubscriptionList.remove(topic);
        byteArraySubscriptionList.remove(topic);
    }
    // #endregion
} // class UrsAI2MQTT