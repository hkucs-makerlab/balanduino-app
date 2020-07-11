package com.makerlab.bt;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.AsyncTask;
import android.os.ParcelUuid;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.Serializable;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.UUID;

public class BluetoothConnect implements Serializable {
    static private String LOG_TAG = BluetoothConnect.class.getSimpleName();
    //
    private BluetoothDevice mBluetoothDevice;
    // classic Bluetooth SPP
    private BluetoothSocket mBluetoothSocket = null;
    private OutputStream mOutputStream;
    private InputStream mInputStream;

    private Activity mActivity;
    //
    private int mPrevChecksum = -1;
    private DisonnectedState mDisonnectedState;
    private ConnectionHandler mConnectionHandler;
    private boolean mIsConnected = false;
    private BluetoothConnect self;
    private Queue<String> recievedPayloadQuene = new LinkedList<>();
    private String mCommandLine = "";

    public BluetoothConnect(Activity activity) {
        mActivity = activity;
        self = this;
    }

    public void setConnectionHandler(ConnectionHandler mConnectionHandler) {
        this.mConnectionHandler = mConnectionHandler;
    }

    public void connectBluetooth(BluetoothDevice bluetoothDevice) {
        if (isConnected()) return;
        this.mBluetoothDevice = bluetoothDevice;
        connectBluetooth();

    }

    public void connectBluetooth() {
        recievedPayloadQuene.clear();
        mCommandLine = "";
        BtSocketConnectAsyncTask btSocketConnectAsyncTask = new BtSocketConnectAsyncTask(mActivity, mBluetoothDevice);
        btSocketConnectAsyncTask.execute();
    }

    public void disconnectBluetooth() {
        if (mDisonnectedState != null) {
            mActivity.unregisterReceiver(mDisonnectedState);
            mDisonnectedState = null;
        }
        //
        try {
            if (mBluetoothSocket != null) {
                if (mOutputStream != null) {
                    mOutputStream.close();
                }
                if (mInputStream != null) {
                    mInputStream.close();
                }
                mBluetoothSocket.close();
            }
        } catch (IOException e) {
            Log.e(LOG_TAG, "disconnectBluetooth(): " + e.toString());
        } finally {
            mBluetoothSocket = null;
            mOutputStream = null;
            mInputStream = null;
        }
        //
        mPrevChecksum = -1;
        mIsConnected = false;
    }

    public String getDeviceAddress() {
        String addr = null;
        if (mBluetoothDevice != null) {
            addr = mBluetoothDevice.getAddress();
        }
        return addr;
    }

    public String getDeviceName() {
        String name = null;
        if (mBluetoothDevice != null) {
            name = mBluetoothDevice.getName();
        }
        return name;
    }


    public boolean isConnected() {
        return mIsConnected;
    }

    public int available() {
        if (mInputStream != null) {  // poll if we have data recevied from bluetooth spp
            try {
                int len = 0;

                len = mInputStream.available();
                if (len > 0) {
                    byte[] buffer = new byte[len];
                    mInputStream.read(buffer);
                    String tmp = new String(buffer);
                    mCommandLine = mCommandLine + tmp;
                    //Log.e(LOG_TAG, "available(): tmp " + tmp);
                }

            } catch (IOException e) {

            }
            if (mCommandLine.contains("\n")) {
                //Log.e(LOG_TAG, "available(): mCommandLine " + mCommandLine);
                recievedPayloadQuene.add(mCommandLine);
                mCommandLine = "";
            }
        }
        return recievedPayloadQuene.size();
    }

    public String read() {
        String data = "";
        try {
            data = recievedPayloadQuene.poll();
        } catch (NoSuchElementException e) {
            data = "";
        }
        return data;
    }

    public boolean write(String buffer) {
        return send(buffer.getBytes());
    }

    public boolean send(byte[] payload) {
        if (!isConnected()) {
            Log.d(LOG_TAG, "send(): connection not set!");
            return false;
        }
        if (payload == null || payload.length == 0) {
            Log.d(LOG_TAG, "send(): invalid payload");
            return false;
        }
        boolean isSuccess = false;
        if (mOutputStream != null) {
            try {
                mOutputStream.write(payload);
                mOutputStream.flush();
                isSuccess = true;
            } catch (IOException e) {
                Log.e(LOG_TAG, "send(): " + e.toString());
                disconnectBluetooth();
            }
        } else {
            Log.e(LOG_TAG, "send(): failed to send, GattCharacteristic or OutputStream is null!");
        }
//        if (isSuccess) {
//            Log.e(LOG_TAG, "send(): sent data " + new String(payload));
//        }

        return isSuccess;
    }


    //  AsyncTask
    class BtSocketConnectAsyncTask extends AsyncTask<Void, Void, String> {
        private Activity activity;
        private BluetoothDevice bluetoothDevice;

        public BtSocketConnectAsyncTask(Activity activity, BluetoothDevice bluetoothDevice) {
            super();
            this.activity = activity;
            this.bluetoothDevice = bluetoothDevice;
        }

        @Override
        protected void onPreExecute() {
            super.onPreExecute();
            if (mConnectionHandler != null) {
                mConnectionHandler.onConnect(self);
            }
        }

        @Override
        protected String doInBackground(Void... voids) {
            int btDeviceType = BluetoothDevice.DEVICE_TYPE_UNKNOWN;
                if (mConnectionHandler != null) {
                    mConnectionHandler.onConnect(self);
                }
                mIsConnected = connectClassicBlueTooth(bluetoothDevice);
                if (mConnectionHandler != null) {
                    if (mIsConnected) {
                        mConnectionHandler.onConnectionSuccess(self);
                    } else {
                        mConnectionHandler.onConnectionFail(self);
                    }
                }
            return null;
        }

/*
        protected void onPostExecute(String result) {

        }
*/

        private boolean connectClassicBlueTooth(BluetoothDevice bluetoothDevice) {
            boolean rc = true;
            try {
                UUID SPPuuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
                mBluetoothSocket = bluetoothDevice.createInsecureRfcommSocketToServiceRecord(SPPuuid);
                mBluetoothSocket.connect();
                mOutputStream = mBluetoothSocket.getOutputStream();
                mInputStream = mBluetoothSocket.getInputStream();
                IntentFilter f2 = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
                IntentFilter f1 = new IntentFilter(BluetoothDevice.ACTION_ACL_DISCONNECTED);
                mDisonnectedState = new DisonnectedState();
                mActivity.registerReceiver(mDisonnectedState, f1);
                mActivity.registerReceiver(mDisonnectedState, f2);
            } catch (IOException e) {
                Log.e(LOG_TAG, "connectClassicBlueTooth(): " + e.toString());
                //e.printStackTrace();
                mOutputStream = null;
                mInputStream = null;
                mBluetoothSocket = null;
                rc = false;
            }
            return rc;
        }

    }//  AsyncTask

    // broadcast receiver for bluetooth disconnect
    private class DisonnectedState extends BroadcastReceiver {
        private final String LOG_TAG = DisonnectedState.class.getSimpleName();

        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (action == null) return;
            if (action.equals(BluetoothDevice.ACTION_ACL_DISCONNECTED)) {
                //Log.e(LOG_TAG, "onReceive() : device disconnected");
                if (mConnectionHandler != null) {
                    mConnectionHandler.onDisconnected(self);
                    disconnectBluetooth();
                }
            } else if (action.equals(BluetoothAdapter.ACTION_STATE_CHANGED)) {
                //Log.e(LOG_TAG, "onReceive() : adaptor state changed");
                final int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, BluetoothAdapter.ERROR);
                switch (state) {
                    case BluetoothAdapter.STATE_OFF:
                        if (mConnectionHandler != null) {
                            mConnectionHandler.onDisconnected(self);
                            disconnectBluetooth();
                        }
                        break;
                    case BluetoothAdapter.STATE_ON:
                        break;
                }
            }
        }
    }

    public interface ConnectionHandler {
        void onConnect(BluetoothConnect self);

        void onConnectionSuccess(BluetoothConnect self);

        void onConnectionFail(BluetoothConnect self);

        void onDisconnected(BluetoothConnect self);
    }
}
