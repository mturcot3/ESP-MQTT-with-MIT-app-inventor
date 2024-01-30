package de.ullisroboterseite.UrsAI2MQTT.messages;

import java.util.Arrays;

/**
 * @brief ByteArrayOutputStream-Erweiterung mit Zugriff auf ein einzelnes Byte.
 */
public class ByteBuffer { // extends ByteArrayOutputStream {
    protected byte[] buf; // Speicher für die Daten.
    protected int count; // Anzahl gültiger Zeichen im Puffer.
    private static final int MAX_ARRAY_SIZE = Integer.MAX_VALUE - 8; // Die maximale Größe des Arrays. Einige VMs
                                                                     // reservieren einige Kontroll-Wörter in einem
                                                                     // Array.

    /**
     * @ brief Creates a new byte array output stream. The buffer capacity is
     * initially 32 bytes, though its size increases if necessary. Initialisiert
     * eine neue Instanz der Klasse ByteBuffer. Die anfängliche Größe ist 32 Byte.
     * Die Größe wird bei Bedarf vergrößert.
     */
    public ByteBuffer() {
        buf = new byte[32];
    }

    /**
     * @brief Liefert das Byte an der angegebenen Position.
     *
     * @param index Postion des Bytes, das ausgeliefert werden soll.
     * @return Das Byte an der Position 'index'.
     */
    public byte byteAt(int index) {
        return buf[index];
    }

    /**
     * Erhöht die Kapazität, um sicherzustellen, dass mindestens die im Argument für
     * die Mindestkapazität angegebene Anzahl von Elementen gespeichert werden kann.
     *
     * @param minCapacity Die gewünschte Mindestkapazität
     */
    private void grow(int minCapacity) {
        int oldCapacity = buf.length;
        int newCapacity = oldCapacity << 1; // Größe verdoppeln
        if (newCapacity < minCapacity)
            newCapacity = minCapacity;
        if (newCapacity > MAX_ARRAY_SIZE)
            newCapacity = MAX_ARRAY_SIZE;
        buf = Arrays.copyOf(buf, newCapacity);
    }

    /**
     * Erhöht die Kapazität, falls erforderlich, um sicherzustellen, dass sie
     * mindestens die im Argument für die Mindestkapazität angegebene Anzahl von
     * Elementen aufnehmen kann.
     *
     * @param minCapacity Die gewünschte Mindestkapazität.
     */
    private void ensureCapacity(int minCapacity) {
        if (minCapacity > buf.length)
            grow(minCapacity);
    }

    /**
     * @brief Fügt das angegebene Byte an die bestehenden Daten an.
     *
     * @param b Das anzufügende Byte.
     */
    public synchronized void add(byte b) {
        ensureCapacity(count + 1);
        buf[count] = b;
        count += 1;
    }

    /**
     * @brief Fügt das angegebene Byte-Array an die bestehenden Daten an.
     *
     * @param b Das anzufügende Byte-Array.
     */
    public synchronized void add(byte[] b) {
        int len = b.length;
        ensureCapacity(count + len);
        System.arraycopy(b, 0, buf, count, len);
        count += len;
    }

    /**
     * @brief Fügt den Inhalt des angegebenen ByteBuffer-Objekts an die bestehenden Daten an.
     *
     * @param b Das anzufügende ByteBuffer-Objekt.
     */
    public synchronized void add(ByteBuffer b) {
        int len = b.count;
        ensureCapacity(count + len);
        System.arraycopy(b.buf, 0, buf, count, len);
        count += len;
    }

    /**
     * @brief Gibt die aktuelle Anzahl gültiger Bytes im Puffer zurück.
     *
     * @return Die aktuelle Anzahl gültiger Bytes im Puffer.
     */
    public synchronized int size() {
        return count;
    }

    /**

     *@brief Erstellt ein neues Byte-Array. Seine Größe ist die aktuelle Anzahl
     * gültiger Bytes im Puffer und der Inhalt des Puffers wurde hinein kopiert.
     * @return Der aktuelle Inhalt Puffers als Byte-Array.
     */
    public synchronized byte[] toByteArray() {
        return Arrays.copyOf(buf, count);
    }
}