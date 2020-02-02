package frc.robot.dataTypes;

import java.util.HashMap;
import java.util.Set;

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

    public Set<K> keySet()  {return this.forward .keySet();}
    public Set<V> valueSet(){return this.backward.keySet();}
}