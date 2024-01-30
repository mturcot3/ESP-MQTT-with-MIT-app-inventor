package de.ullisroboterseite.UrsAI2MQTT;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.concurrent.locks.ReentrantLock;

import android.util.*;
import de.ullisroboterseite.UrsAI2MQTT.messages.*;

class MqttMessageQueue extends LinkedList<MsgMqtt> {
    private static final long serialVersionUID = -3945743990902593237L;
    static final String LOG_TAG = UrsAI2MQTT.LOG_TAG;

    final ReentrantLock QueueLock = new ReentrantLock();

    String name;

    MqttMessageQueue(String name) {
        this.name = name;
    }

    void lock() {
        QueueLock.lock();
    }

    void unlock() {
        QueueLock.unlock();
    }

    /**
     * Fügt der Liste eine Nachricht hinzu.
     * @param msgMqtt Die Nachricht, die hinzugefügt werden soll.
     * @return
     */
    public boolean addSync(MsgMqtt msgMqtt) {
        lock();
        boolean rtc = add(msgMqtt);
        Log.v(LOG_TAG, name + "> Msg added: " + msgMqtt.toString());
        unlock();
        return rtc;
    }

    /**
     * Löscht eine Nachricht aus der Liste.
     * @param type Der Nachricht, die gelöscht werden soll.
     * @param id PacketIdentifier der zu löschenden Nachricht
     * @return true, wenn eine passende Nachricht gefunden wurde.
     */
    MsgMqtt removeSync(MsgType type, int id) {
        lock();
        Iterator<MsgMqtt> iterator = iterator();
        while (iterator.hasNext()) {
            MsgMqtt m = iterator.next();
            if (m.getType() == type) {
                if (m.packetIdentifier == id) {
                    iterator.remove();
                    Log.v(LOG_TAG, name + "> Msg removed: " + m.toString());
                    unlock();
                    return m;
                }
            }
        }
        unlock();
        return null;
    }

    /**
     * Löscht eine Nachricht eines Typs aus der Liste.
     * @param type Der Nachricht, die gelöscht werden soll.
     * @return true, wenn eine passende Nachricht gefunden wurde.
     */
    void remove(MsgType type) {
        lock();
        Iterator<MsgMqtt> iterator = iterator();
        while (iterator.hasNext()) {
            MsgMqtt m = iterator.next();
            if (m.getType() == type) {
                iterator.remove();
                Log.v(LOG_TAG, name + "> Msg removed: " + m.toString());
            }
        }
        unlock();
    }

    /**
     * Löscht alle Einträge aus der Liste.
     */
    void clearSync() {
        lock();
        clear();
        Log.v(LOG_TAG, name + "> Queue cleared");
        unlock();
    }

    boolean find(int id) {
        lock();
        // InboundQueue prüfen
        boolean found = false;

        Iterator<MsgMqtt> it = iterator();
        while (it.hasNext()) {
            MsgMqtt m = it.next();
            if (m.packetIdentifier == id)
                found = true;
            break; // while
        }
        unlock();
        return found;
    }
}