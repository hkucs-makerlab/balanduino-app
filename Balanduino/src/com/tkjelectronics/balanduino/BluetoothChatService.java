/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.tkjelectronics.balanduino;

import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;

import com.makerlab.bt.BluetoothConnect;
/**
 * This class does all the work for setting up and managing Bluetooth
 * connections with other devices. It has a thread for connecting with a device,
 * and a thread for performing data transmissions when connected.
 */
public class BluetoothChatService {
    // Debugging
    private static final String TAG = "BluetoothChatService";
    private static final boolean D = BalanduinoActivity.D;

    // Member fields
    private BluetoothConnect mBluetoothConnect;
    private final Handler mHandler;
    private ConnectedThread mConnectedThread;
    private int mState;

    // Constants that indicate the current connection state
    public static final int STATE_NONE = 0; // we're doing nothing
    public static final int STATE_CONNECTED = 1; // now connected to a remote device

    private boolean stopReading; // This is used to stop it from reading on the inputStream

    /**
     * Constructor. Prepares a new BluetoothChat session.
     *
     * @param handler A Handler to send messages back to the UI Activity
     */
    public BluetoothChatService(Handler handler, BluetoothConnect conn) {
        mBluetoothConnect = conn;
        mState = STATE_NONE;
        mHandler = handler;
    }

    /**
     * Set the current state of the chat connection
     *
     * @param state An integer defining the current connection state
     */

    private synchronized void setState(int state) {
        if (D)
            Log.d(TAG, "setState() " + mState + " -> " + state);
        mState = state;
        // Give the new state to the Handler so the UI Activity can update
        mHandler.obtainMessage(BalanduinoActivity.MESSAGE_STATE_CHANGE, state,
                -1).sendToTarget();
    }

    /**
     * Return the current connection state.
     */
    public synchronized int getState() {
        return mState;
    }

    public synchronized void connected() {
        // Start the thread to manage the connection and perform transmissions
        mConnectedThread = new ConnectedThread();
        mConnectedThread.start();

        // Send the name of the connected device back to the UI Activity
        Message msg = mHandler.obtainMessage(BalanduinoActivity.MESSAGE_DEVICE_NAME);
        Bundle bundle = new Bundle();
        bundle.putString(BalanduinoActivity.DEVICE_NAME, mBluetoothConnect.getDeviceName());
        msg.setData(bundle);
        mHandler.sendMessage(msg);

        setState(STATE_CONNECTED);
    }

    /**
     * Stop all threads
     */
    public synchronized void stop() {
        if (D)
            Log.d(TAG, "stop");
        stopReading = true;

        if (mConnectedThread != null) {
            mConnectedThread.cancel();
            mConnectedThread = null;
        }
        setState(STATE_NONE);
        mBluetoothConnect.disconnectBluetooth();
    }

    /**
     * Write to the ConnectedThread in an unsynchronized manner
     *
     * @param out The bytes to write
     * @see ConnectedThread#write(byte[])
     */
    public synchronized void write(byte[] out) {
        // Create temporary object
        //ConnectedThread r;
        // Synchronize a copy of the ConnectedThread
/*        synchronized (this) {
            if (mState != STATE_CONNECTED)
                return;
            r = mConnectedThread;
        }*/
        // Perform the write unsynchronized
        //r.write(out);
        mBluetoothConnect.send(out);
    }

    public synchronized void write(String string) {
        write(string.getBytes());
    }

    /**
     * This thread runs during a connection with a remote device. It handles all
     * incoming and outgoing transmissions.
     */
    private class ConnectedThread extends Thread {
        private static final String TAG = "ConnectedThread";

        public ConnectedThread() {

        }

        public void run() {
            if (D)
                Log.i(TAG, "BEGIN mConnectedThread");

            while (!stopReading) {
                if (mBluetoothConnect.available() > 0) { // Check if new data is available
                    String readMessage = mBluetoothConnect.read(); // Read from the InputStream
                    String[] splitMessage = readMessage.split(",");
                    if (D) {
                        Log.e(TAG, "Received string: " + readMessage);
                        for (int i = 0; i < splitMessage.length; i++)
                            Log.e(TAG, "splitMessage[" + i + "]: " + splitMessage[i]);
                    }

                    for (int i = 0; i < splitMessage.length; i++)
                        splitMessage[i] = splitMessage[i].trim(); // Trim message

                    if (splitMessage[0].equals(BalanduinoActivity.responsePIDValues) &&
                            splitMessage.length == BalanduinoActivity.responsePIDValuesLength) {
                        BalanduinoActivity.pValue = splitMessage[1];
                        BalanduinoActivity.iValue = splitMessage[2];
                        BalanduinoActivity.dValue = splitMessage[3];
                        BalanduinoActivity.targetAngleValue = splitMessage[4];
                        BalanduinoActivity.newPIDValues = true;
                        mHandler.obtainMessage(BalanduinoActivity.MESSAGE_READ).sendToTarget(); // Send message back to the UI Activity

                    } else if (splitMessage[0].equals(BalanduinoActivity.responseSettings) &&
                            splitMessage.length == BalanduinoActivity.responseSettingsLength) {
                        BalanduinoActivity.backToSpot = splitMessage[1].equals("1");
                        BalanduinoActivity.maxAngle = Integer.parseInt(splitMessage[2]);
                        BalanduinoActivity.maxTurning = Integer.parseInt(splitMessage[3]);

                    } else if (splitMessage[0].equals(BalanduinoActivity.responseInfo) &&
                            splitMessage.length == BalanduinoActivity.responseInfoLength) {
                        BalanduinoActivity.firmwareVersion = splitMessage[1];
                        BalanduinoActivity.eepromVersion = splitMessage[2];
                        BalanduinoActivity.mcu = splitMessage[3];
                        BalanduinoActivity.newInfo = true;
                        mHandler.obtainMessage(BalanduinoActivity.MESSAGE_READ).sendToTarget(); // Send message back to the UI Activity

                    } else if (splitMessage[0].equals(BalanduinoActivity.responseStatus) &&
                            splitMessage.length == BalanduinoActivity.responseStatusLength) {
                        BalanduinoActivity.batteryLevel = splitMessage[1];
                        BalanduinoActivity.runtime = Double.parseDouble(splitMessage[2]);
                        BalanduinoActivity.newStatus = true;
                        mHandler.obtainMessage(BalanduinoActivity.MESSAGE_READ).sendToTarget(); // Send message back to the UI Activity

                    } else if (splitMessage[0].equals(BalanduinoActivity.responseKalmanValues) &&
                            splitMessage.length == BalanduinoActivity.responseKalmanValuesLength) {
                        BalanduinoActivity.Qangle = splitMessage[1];
                        BalanduinoActivity.Qbias = splitMessage[2];
                        BalanduinoActivity.Rmeasure = splitMessage[3];
                        BalanduinoActivity.newKalmanValues = true;
                        mHandler.obtainMessage(BalanduinoActivity.MESSAGE_READ).sendToTarget(); // Send message back to the UI Activity

                    } else if (splitMessage[0].equals(BalanduinoActivity.responseIMU) &&
                            splitMessage.length == BalanduinoActivity.responseIMULength) {
                        BalanduinoActivity.accValue = splitMessage[1];
                        BalanduinoActivity.gyroValue = splitMessage[2];
                        BalanduinoActivity.kalmanValue = splitMessage[3];
                        BalanduinoActivity.newIMUValues = true;
                        mHandler.obtainMessage(BalanduinoActivity.MESSAGE_READ).sendToTarget(); // Send message back to the UI Activity

                    } else if (splitMessage[0].equals(BalanduinoActivity.responsePairConfirmation) &&
                            splitMessage.length == BalanduinoActivity.responsePairConfirmationLength) {
                        BalanduinoActivity.pairingWithDevice = true;
                        mHandler.obtainMessage(BalanduinoActivity.MESSAGE_READ).sendToTarget(); // Send message back to the UI Activity
                    }
                }

            }
        }

        /**
         * Write to the connected OutStream.
         *
         * @param buffer The bytes to write
         */
        public void write(byte[] buffer) {
            mBluetoothConnect.send(buffer);
        }

        public void cancel() {
            stopReading = true;
        }
    }
}