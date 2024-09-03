package org.firstinspires.ftc.teamcode.optimizations;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.util.geometry.Pose;


/**
 * A buffer of predetermined size. New values will override the oldest value if capacity is reached.
 */
public class RingBuffer {
    private final Pose[] array;
    private int head = -1; // read head, aka index of most recent element

    public RingBuffer(int capacity) {
        array = new Pose[capacity];
    }

    /**
     * Reads elements from buffer sorted by recency.
     * @param index Should be between 0 and capacity, where 0 returns most recently added element.
     * @return null if not added
     */
    public Pose read(int index) {
        // throw if out of bounds
        if (index < 0 || index >= array.length) {
            throw new IndexOutOfBoundsException();
        }

        // if empty, return null
        if (head == -1) {
            return null;
        }

        // otherwise, return element
        return array[(head - index + array.length) % array.length];
    }

    /**
     * Adds elements into ring buffer
     * @param element a new element to add into buffer
     */
    public void put(@NonNull Pose element) {
        head = (head + 1) % array.length;
        array[head] = element;
    }

    /**
     * Returns initialization capacity of ring buffer
     */
    public int capacity() {
        return array.length;
    }

    /**
     * Returns the number of elements in the buffer.
     * This number can never exceed capacity.
     */
    public int size() {
        int size = 0;
        for (Pose element : array) {
            if (element != null) size++;
            else break;
        }
        return size;
    }


    public boolean isFull() {
        for (Pose element : array) {
            if (element == null) return false;
        }
        return true;
    }

    public Pose[] getOldestToLatest() {
        int size = size();
        Pose[] orderedArray = new Pose[size];

        int startIndex = (head + 1) % array.length;

        for (int i = 0; i < size; i++) {
            int index = (startIndex + i) % array.length;
            orderedArray[i] = array[index];
        }

        return orderedArray;
    }

}