package com.makerlab.bt;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothClass;
import android.bluetooth.BluetoothDevice;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.ParcelUuid;
import android.support.v4.app.ActivityCompat;
import android.util.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

public class BluetoothScan {
    static public final int REQUEST_BT_PERMISSION = 39;
    static public final int REQUEST_BT_ENABLE = 22;
    static private String LOG_TAG =BluetoothScan.class.getSimpleName();

    //
    private BluetoothAdapter mBluetoothAdapter;
    private ClassicBtDiscoveryCallback mDiscoveryBroadcastReceiver;
    private ClassicBtDiscoveryResultCallback mClassBtDiscveryResultCallback;
    private boolean mPermssionGranted = false;
    private Activity activity;
    private Map<String, BluetoothDevice> mBluetoothDeviceMap;
    private ResultHandler handler;
    private boolean registeredReceiver=false;



    public BluetoothScan(Activity activity) {
        this.activity = activity;
        mBluetoothDeviceMap = new HashMap<>();
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        mDiscoveryBroadcastReceiver = new ClassicBtDiscoveryCallback();
        mClassBtDiscveryResultCallback = new ClassicBtDiscoveryResultCallback();
    }

    public void setResultHandler(ResultHandler handler) {
        this.handler = handler;
    }

    public BluetoothDevice getBluetoothDevice(String address) {
        BluetoothDevice bluetoothDevice = null;
        BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (!mBluetoothAdapter.isEnabled()) {
            Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            activity.startActivityForResult(intent, REQUEST_BT_ENABLE);
        } else {
            requestBluetoothAccessPermission();
            bluetoothDevice = mBluetoothAdapter.getRemoteDevice(address);
        }
        return bluetoothDevice;
    }

    public boolean start() {
        if (!mBluetoothAdapter.isEnabled()) {
            Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            activity.startActivityForResult(intent, REQUEST_BT_ENABLE);
            return false;
        }
        //
        requestBluetoothAccessPermission();
        // for paired devices
        addClassicBtPairedDevices();
        // for classic SPP devices
        if (!mBluetoothAdapter.isDiscovering()) {
            activity.registerReceiver(mDiscoveryBroadcastReceiver,
                    new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_STARTED));
            activity.registerReceiver(mDiscoveryBroadcastReceiver,
                    new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_FINISHED));
            activity.registerReceiver(mClassBtDiscveryResultCallback,
                    new IntentFilter(BluetoothDevice.ACTION_FOUND));
            mBluetoothAdapter.startDiscovery();
            registeredReceiver=true;
        }


        return true;
    }

    public void stop() {
        if (!mBluetoothAdapter.isEnabled()) return;

        // cancel Bluetooth Class Scanning
        if (mBluetoothAdapter.isDiscovering()) {
            mBluetoothAdapter.cancelDiscovery();
        }
        if (registeredReceiver) {
            registeredReceiver=false;
            activity.unregisterReceiver(mDiscoveryBroadcastReceiver);
            activity.unregisterReceiver(mClassBtDiscveryResultCallback);
        }
    }

    public Map<String, BluetoothDevice> getmBluetoothDevices() {
        return mBluetoothDeviceMap;
    }

    private void addClassicBtPairedDevices() {
        Set<BluetoothDevice> bondedDevices = mBluetoothAdapter.getBondedDevices();
        Iterator<BluetoothDevice> loop = bondedDevices.iterator();
        while (loop.hasNext()) {
            BluetoothDevice remoteDevice = loop.next();
            BluetoothClass btClass = remoteDevice.getBluetoothClass();
            if (btClass.getMajorDeviceClass() == BluetoothClass.Device.Major.UNCATEGORIZED) {
                mBluetoothDeviceMap.put(remoteDevice.getAddress(), remoteDevice);
                if (handler != null) {
                    handler.setResult(remoteDevice);
                }
            }
        }
    }

    private void requestBluetoothAccessPermission() {
        mPermssionGranted = ActivityCompat.checkSelfPermission(activity, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED;
        if (!mPermssionGranted) {
            String[] permissions = new String[]{
                    Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.ACCESS_FINE_LOCATION
            };
            ActivityCompat.requestPermissions(activity, permissions, REQUEST_BT_PERMISSION);
        }
    }

    //
    public class ClassicBtDiscoveryCallback extends BroadcastReceiver {
        private String LOG_TAG =ClassicBtDiscoveryCallback.class.getSimpleName();
        @Override
        public void onReceive(Context context, Intent intent) {
            String intentActon = intent.getAction();

            if (BluetoothAdapter.ACTION_DISCOVERY_STARTED.equals(intentActon)) {
                Log.e(LOG_TAG, "onReceive() - Discovery Started...");
            } else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(intentActon)) {
                Log.e(LOG_TAG, "onReceive() - Discovery Complete.");
                if (handler != null) {
                    handler.onPostResult();
                }
            }
        }
    }

    //
    public class ClassicBtDiscoveryResultCallback extends BroadcastReceiver {
        private String LOG_TAG =ClassicBtDiscoveryResultCallback.class.getSimpleName();
        @Override
        public void onReceive(Context context, Intent intent) {
//            String remoteDeviceName =
//                    intent.getStringExtra(BluetoothDevice.EXTRA_NAME);
            BluetoothDevice remoteDevice =
                    intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
            if (remoteDevice != null) {
                mBluetoothDeviceMap.put(remoteDevice.getAddress(), remoteDevice);
                BluetoothClass btClass = remoteDevice.getBluetoothClass();
                if (btClass.getMajorDeviceClass() == BluetoothClass.Device.Major.UNCATEGORIZED) {
                    if (handler != null) {
                        handler.setResult(remoteDevice);
                    }
                }
            }
        }
    }

    public interface ResultHandler {
        public void setResult(BluetoothDevice bluetoothDevice);

        public void onPostResult();
    }
}
