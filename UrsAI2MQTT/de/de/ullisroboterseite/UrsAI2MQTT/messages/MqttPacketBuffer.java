package de.ullisroboterseite.UrsAI2MQTT.messages;

import de.ullisroboterseite.UrsAI2MQTT.MqttChannel;
import de.ullisroboterseite.UrsAI2MQTT.MqttClient;
import de.ullisroboterseite.UrsAI2MQTT.MqttErrorCode;
import de.ullisroboterseite.UrsAI2MQTT.MqttException;

/**
 * Repräsentiert den Byte-Puffer für eine MQTT-Nachricht
*/
public class MqttPacketBuffer {
  private   ByteBuffer dataBuffer = new ByteBuffer();
    private byte PacketType;

    /**
     * \brief Initialisiert eine neue Instanz der MqttPacketBuffer-Klasse
     *
     * @param MqqtPacketType Der MQTT-Paket-Typ
     */
    public MqttPacketBuffer(byte MqqtPacketType) {
        PacketType = MqqtPacketType;
    }

    private MqttPacketBuffer() { // gesperrter Default-Konstruktor

    }

    // #region Append
    /**
     * Fügt den String 'Source' hinzu. Das Format wird durch
     * UrsMqttClient.Charset bestimmt. An die aktuelle Position wird zunächst die
     * Länge des Strings in Byte eingefügt, danach der Text.
     *
     * @param source Text, der angefügt werden soll.
     */
    public void append(String source) {
        byte[] b = source.getBytes(MqttClient.Charset);
        dataBuffer.add((byte) (b.length >> 8));
        dataBuffer.add((byte) (b.length & 0xFF));
        dataBuffer.add(b);
    }

    /**
     * \brief Fügt das angegebene Byte-Array an.
     *
     * @param source Das anzufügende Byte-Array.
     */
    public void append(byte[] source) {
        dataBuffer.add(source);
    }

    /**
     * \brief Fügt das angegebene Byte an.
     *
     * @param source Das anzufügende Byte.
     */
    public void append(byte source) {
        dataBuffer.add(source);
    }
    // #endregion

    // #region getMqttPacketData
    // Baut den Fixed-Header auf.
    private ByteBuffer getFixedHeader() {
        ByteBuffer fh = new ByteBuffer();
        fh.add(PacketType);
        int len = dataBuffer.size();
        byte digit;

        do {
            digit = (byte) (len % 128);
            len = len / 128;
            if (len > 0) {
                digit |= 0x80;
            }
            fh.add(digit);
        } while (len > 0);
        return fh;
    }

    /**
     * \brief Liefert den zusammengebauten Nachrichten-Puffer als Byte-Array.
     *
     * @return Der zusammengebaute Nachrichten-Puffer als Byte-Array.
     */
    public byte[] getMqttPacketData() {
        ByteBuffer o = new ByteBuffer();
        o.add(getFixedHeader());
        o.add(dataBuffer);
        // paketBuffer = o.toByteArray();
        return o.toByteArray();
    }
    // #endregion

    // #region FromStream
    /**
     * Erstellt ein MqttPacketBuffer-Objekt aus den Daten, die vom angegebenen
     * Client gelesen werden.
     *
     * @param mqttChannel Client, der die Daten zur Verfügung stellt.
     * @return Das aufbereitete Paket.
     * @throws MqttException Die Daten konnten nicht wie erwartet gelesen werden.
     */
    public static MqttPacketBuffer fromStream(MqttChannel mqttChannel) throws MqttException {
        int pos = 1;
        byte digit = 0;
        byte byteRead;
        int multiplier = 1;
        int length = 0;
        MqttPacketBuffer mpb = new MqttPacketBuffer();

        // Paket-Typ einlesen
        byteRead = mqttChannel.readByteTimeout();
        mpb.PacketType = byteRead;

        // Paket-Länge einlesen
        do {
            if (pos == 5) {
                throw new MqttException(MqttErrorCode.InvalidPaketFormat);
            }
            byteRead = mqttChannel.readByteTimeout();
            digit = (byte) byteRead;

            length += (digit & 127) * multiplier;
            multiplier *= 128;
        } while ((digit & 128) != 0);

        // Daten einlesen
        for (int i = 0; i < length; i++) {
            mpb.dataBuffer.add(mqttChannel.readByteTimeout());
        }
        return mpb;
    }
    // #endregion

    /**
     * @brief Liefert den MQTT-Paket-Typ.
     * @note Dies sind die obersten 4 Bit. Flags (die unteren 4 bits) werden
     *       abgeschnitten.
     * @return Der MQTT-Paket-Typ
     */
    public byte getType() {
        return (byte) (PacketType & 0xF0);
    }

    /**
     * @brief Liefert die Länge des Datenteils des Pakets.
     * @return Die Länge des Datenteils des Pakets.
     */
    public int getDataLength() {
        return dataBuffer.size();
    }

  public int getFlags() {
        return (byte) (PacketType & 0x0F);
    }

    public void setFlags(int b) {
        PacketType = (byte) ((PacketType & 0xF0) | (b & 0x0F));
    }

    int readPosition = 0;
    // Liefert das nächste Byte als Wert zwischen 0 und 255 (unsigned).
    public int getByteValue() {
        int a = dataBuffer.byteAt(readPosition++);
        if (a < 0) a += 256;
        return a;
    }
    // Liefert das nächste Byte als Wert zwischen -128 und 127 (signed).
    public byte getByte() {
        return dataBuffer.byteAt(readPosition++);
    }
}