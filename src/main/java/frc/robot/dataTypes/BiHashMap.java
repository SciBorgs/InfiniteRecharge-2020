package frc.robot.helpers;

import java.util.HashMap;

public class BiHashMap<K, V> {
    private HashMap<K, V> forward;
    private HashMap<V, K> backward;

    public BiHashMap() {
        this.forward = new HashMap<>();
        this.backward = new HashMap<>();
    }

    public void put(K key, V val) {
        this.forward.put(key, val);
        this.backward.put(val, key);
    }

    public V getForward(K key){return this.forward.get(key);}
    public K getBackward(V key){return this.backward.get(key);}
}