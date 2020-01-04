package frc.robot.helpers;

import java.util.ArrayList;
import java.util.List;

public class Deque<T> {
    private List<T> list;
    public int maxLength, head;

    public Deque(int maxLength) {
        this.list = new ArrayList<>(maxLength);
        this.maxLength = maxLength;
        this.head = -1;
    }

    public Deque(Iterable<T> iterable, int maxLength) {
        this(maxLength);
        for (T element: iterable) {
            add(element);
        }
    }

    public void add(T element) {
        this.head = (this.head + 1) % this.maxLength;
        if (this.list.size() == this.maxLength){this.list.set(this.head, element);} 
        else{this.list.add(element);}
    }

    public void set(int index, T element) {
        this.list.set(Math.floorMod(this.head - index, this.list.size()), element);
    }

    public T get(int index) {
        if (index >= 0 && index < this.list.size()) {
            return this.list.get(Math.floorMod(this.head - index, this.list.size()));
        }
        throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + this.list.size());
    }

    public void remove(int index) {
        this.list.remove(index);
        this.head = Math.floorMod(this.head - 1, this.list.size());
    }

    public List<T> getArrayList() {
        return this.list;
    }

    public int getSize() {
        return this.list.size();
    }

    @Override
    public Deque<T> clone() {
        Deque<T> clone = new Deque<>(this.list, this.maxLength);
        clone.head = this.head;
        return clone;
    }
}