package frc.robot.dataTypes;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Deque<T> implements Iterable<T> {
    // For us, if you add an element to a deque, and do .get(0) you will get that
    // Once you hit the maximum length, you will effectively also remove the last
    // element (the element that's been in the deque for the longest)
    // To make adding O(1), when it is the max length, we shift our "starting
    // point", and just set an element of the arraylist
    private List<T> list;
    public int maxLength, head;

    public Deque(int maxLength) {
        this.list = new ArrayList<>(maxLength);
        this.maxLength = maxLength;
        this.head = -1;
    }

    public Deque(Iterable<T> iterable, int maxLength) {
        this(maxLength);
        for (T element : iterable) {
            add(element);
        }
    }

    private int normalizeIndex(int index) {
        if (this.list.size() == 0) {
            return this.head;
        } else {
            return Math.floorMod(this.head - index, this.list.size());
        }
    }

    public void add(T element) {
        this.head = (this.head + 1) % this.maxLength;
        if (this.list.size() == this.maxLength) {
            this.list.set(this.head, element);
        } else {
            this.list.add(element);
        }
    }

    public void set(int index, T element) {
        this.list.set(normalizeIndex(index), element);
    }

    public T get(int index) {
        if (index >= 0 && index < this.list.size()) {
            return this.list.get(normalizeIndex(index));
        }
        throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + this.list.size());
    }

    public T last() {
        return get(this.list.size() - 1);
    }

    public T first() {
        return get(0);
    }

    public void remove(int index) {
        this.list.remove(normalizeIndex(index));
        this.head = normalizeIndex(1);
    }

    public List<T> getArrayList() {
        return this.list;
    }

    public int getSize() {
        return this.list.size();
    }

    public boolean isEmpty() {
        return this.list.isEmpty();
    }

    @Override
    public Deque<T> clone() {
        Deque<T> clone = new Deque<>(this.list, this.maxLength);
        clone.head = this.head;
        return clone;
    }

    @Override
    public Iterator<T> iterator() {
        return new DequeIterator<T>(this);
    }

    public Deque<T> reversed() {
        Deque<T> deq = new Deque<T>(this.maxLength);
        for (T t : this) {
            deq.add(t);
        }
        return deq;
    }
}

class DequeIterator<T> implements Iterator<T> {

    private Deque<T> deque;

    public DequeIterator(Deque<T> deque) {
        this.deque = deque.clone();
    }

    @Override
    public boolean hasNext() {
        return !this.deque.isEmpty();
    }

    @Override
    public T next() {
        T el = this.deque.get(0);
        remove();
        return el;
    }

    @Override
    public void remove() {
        this.deque.remove(0);
    }
}