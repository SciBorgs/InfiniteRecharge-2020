package frc.robot.helpers;

import frc.robot.Utils;

import java.util.ArrayList;

public class QueuedArrayList<T> {

    // .size() -> O(n)
    // .

    private ArrayList<T> data;
    private int maxLength;
    private int firstIndex;

    public QueuedArrayList(int maxLength) {
        this.data = new ArrayList<T>();
        this.maxLength = maxLength;
        this.firstIndex = 0;
    }
    public int getMaxLength(){return this.maxLength;}
    public void setMaxLength(int maxLength){this.maxLength = maxLength;}
    public void setFirstIndex(int firstIndex){this.firstIndex = firstIndex;}

    public static<T> QueuedArrayList<T> fromIterable(Iterable<T> iterable, int maxLength){
        QueuedArrayList<T> queued = new QueuedArrayList<T>(maxLength);
        ArrayList<T> reversed = new ArrayList<T>();
        for(T el : iterable){reversed.add(el);}
        for(T el : reversed){queued.add(el);}
        return queued;
    }

    public int size(){return this.data.size();}
    public void set(int i, T el){this.data.set(convertIndex(i), el);}
    public void remove(int i){
        this.data.remove(i);
        this.firstIndex = this.firstIndex - 1;
    }

    public ArrayList<T> toArrayList(){
        ArrayList<T> arr = new ArrayList<T>();
        for(int i = 0; i < arr.size(); i++){
            arr.add(this.data.get(convertIndex(i)));
        }
        return arr;
    }

    private int normalizeIndex(int index) {
        return Utils.bringInRange(index, 0, this.data.size());
    }
    private int convertIndex(int index){
        return normalizeIndex(this.firstIndex - index);
    }

    public void add(T el) {
        if (this.data.size() < this.maxLength) {
            this.data.add(Utils.bringInRange(this.firstIndex - 1, 0, this.data.size() + 1), el);
        } else {
            this.data.set(this.firstIndex, el);
        }
        this.firstIndex = normalizeIndex(this.firstIndex + 1);
    }

    public T get(int index){
        if (index >= 0 && index < this.data.size()) {
            return this.data.get(convertIndex(this.firstIndex));
        } else {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + this.data.size());
        }
    }

    @Override
    public QueuedArrayList<T> clone(){
        QueuedArrayList<T> clone = fromIterable((ArrayList<T>) this.data.clone(), this.maxLength);
        clone.setFirstIndex(this.firstIndex);
        return clone;
    }

}