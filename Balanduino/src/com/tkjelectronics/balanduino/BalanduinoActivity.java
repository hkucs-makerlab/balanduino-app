/*************************************************************************************
 * Copyright (C) 2012-2014 Kristian Lauszus, TKJ Electronics. All rights reserved.
 *
 * This software may be distributed and modified under the terms of the GNU
 * General Public License version 2 (GPL2) as published by the Free Software
 * Foundation and appearing in the file GPL2.TXT included in the packaging of
 * this file. Please note that GPL2 Section 2[b] requires that all works based
 * on this software must also be made publicly available under the terms of
 * the GPL2 ("Copyleft").
 *
 * Contact information
 * -------------------
 *
 * Kristian Lauszus, TKJ Electronics
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  kristianl@tkjelectronics.com
 *
 ************************************************************************************/

package com.tkjelectronics.balanduino;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.content.pm.PackageManager.NameNotFoundException;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.preference.PreferenceManager;
import android.support.design.widget.TabLayout;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.inputmethod.InputMethodManager;
import android.widget.Toast;

import com.makerlab.bt.BluetoothConnect;
import com.makerlab.bt.BluetoothScan;
import com.makerlab.ui.BluetoothDevListActivity;


public class BalanduinoActivity extends AppCompatActivity implements
        TabLayout.OnTabSelectedListener, BluetoothConnect.ConnectionHandler {
    public static final boolean D = BuildConfig.DEBUG; // This is automatically set when building
    // Message types sent from the BluetoothChatService Handler
    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    /*
    public static final int MESSAGE_DEVICE_NAME = 3;
    public static final int MESSAGE_DISCONNECTED = 4;
    public static final int MESSAGE_RETRY = 5;
    */
    // Key names received from the BluetoothChatService Handler
    public static final String DEVICE_NAME = "device_name";
    public static final String TOAST = "toast";
    public final static String getPIDValues = "GP;";
    public final static String getSettings = "GS;";
    public final static String getInfo = "GI;";
    public final static String getKalman = "GK;";
    public final static String setPValue = "SP,";
    public final static String setIValue = "SI,";
    public final static String setDValue = "SD,";
    public final static String setKalman = "SK,";
    public final static String setTargetAngle = "ST,";
    public final static String setMaxAngle = "SA,";
    public final static String setMaxTurning = "SU,";
    public final static String setBackToSpot = "SB,";
    public final static String imuBegin = "IB;";
    public final static String imuStop = "IS;";
    public final static String statusBegin = "RB;";
    public final static String statusStop = "RS;";
    public final static String sendStop = "CS;";
    public final static String sendIMUValues = "CM,";
    public final static String sendJoystickValues = "CJ,";
    public final static String sendPairWithWii = "CPW;";
    public final static String sendPairWithPS4 = "CPP;";
    public final static String restoreDefaultValues = "CR;";
    public final static String responsePIDValues = "P";
    public final static String responseKalmanValues = "K";
    public final static String responseSettings = "S";
    public final static String responseInfo = "I";
    public final static String responseIMU = "V";
    public final static String responseStatus = "R";
    public final static String responsePairConfirmation = "PC";
    public final static int responsePIDValuesLength = 5;
    public final static int responseKalmanValuesLength = 4;
    public final static int responseSettingsLength = 4;
    public final static int responseInfoLength = 4;
    public final static int responseIMULength = 4;
    public final static int responseStatusLength = 3;
    public final static int responsePairConfirmationLength = 1;
    private static final String LOG_TAG = BalanduinoActivity.class.getSimpleName();
    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE = 1;
    private static final int REQUEST_ENABLE_BT = 2;
    public static Activity activity;
    public static Context context;
    // Member object for the chat services
    public static BluetoothChatService mChatService = null;
    public static SensorFusion mSensorFusion = null;
    public static boolean stopRetrying;
    public static int currentTabSelected;
    public static String accValue = "";
    public static String gyroValue = "";
    public static String kalmanValue = "";
    public static boolean newIMUValues;
    public static String Qangle = "";
    public static String Qbias = "";
    public static String Rmeasure = "";
    public static boolean newKalmanValues;
    public static String pValue = "";
    public static String iValue = "";
    public static String dValue = "";
    public static String targetAngleValue = "";
    public static boolean newPIDValues;
    public static boolean backToSpot;
    public static int maxAngle = 8; // Eight is the default value
    public static int maxTurning = 20; // Twenty is the default value
    public static String appVersion;
    public static String firmwareVersion;
    public static String eepromVersion;
    public static String mcu;
    public static boolean newInfo;
    public static String batteryLevel;
    public static double runtime;
    public static boolean newStatus;
    public static boolean pairingWithDevice;
    public static boolean buttonState;
    public static boolean joystickReleased;
    private static Toast mToast;
    // Local Bluetooth adapter
    private BluetoothAdapter mBluetoothAdapter = null;
    private BluetoothHandler mBluetoothHandler = null;
    private BluetoothDevice mBtDevice; // The BluetoothDevice object
    private boolean mBtSecure; // If it's a new device we will pair with the device
    private CustomViewPager mViewPager;
    //
    private BluetoothConnect mBluetoothConnect;
    private BluetoothScan mBluetoothScan;
    //
    private MenuItem menuItemConnect;
    private SharedPreferences mPreferences;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        activity = this;
        context = getApplicationContext();

        if (!getResources().getBoolean(R.bool.isTablet))
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT); // Set portrait mode only - for small screens like phones
        else {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR2)
                setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_FULL_USER); // Full screen rotation
            else
                setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_FULL_SENSOR); // Full screen rotation
            new Handler().postDelayed(new Runnable() { // Hack to hide keyboard when the layout it rotated
                @Override
                public void run() {
                    InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE); // Hide the keyboard - this is needed when the device is rotated
                    imm.hideSoftInputFromWindow(getWindow().getDecorView().getApplicationWindowToken(), 0);
                }
            }, 1000);
        }

        setContentView(R.layout.activity_main);

        // Get local Bluetooth adapter
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR2)
            mBluetoothAdapter = ((BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE)).getAdapter();
        else
            mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported
        if (mBluetoothAdapter == null) {
            if (D) {
                Log.d(LOG_TAG, "No bluetooth adapter!");
            }
            finish();
            return;
        }

        mBluetoothConnect = new BluetoothConnect(this);
        mBluetoothConnect.setConnectionHandler(this);
        mBluetoothScan = new BluetoothScan(this);
        //
        mBluetoothHandler = new BluetoothHandler(this);
        mChatService = new BluetoothChatService(mBluetoothHandler, mBluetoothConnect);
        //
        // get sensorManager and initialize sensor listeners
        SensorManager mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mSensorFusion = new SensorFusion(getApplicationContext(), mSensorManager);
        //
        TabLayout tabLayout = findViewById(R.id.tab_layout);
        tabLayout.addOnTabSelectedListener(this); // listen to detect tab selection
        TabLayout.TabLayoutOnPageChangeListener pageListener = new TabLayout.TabLayoutOnPageChangeListener(tabLayout);
        // Create the adapter that will return a fragment for each of the primary sections of the app.
        ViewPagerAdapter mViewPagerAdapter = new ViewPagerAdapter(getApplicationContext(), getSupportFragmentManager());

        // Set up the ViewPager with the adapter.
        mViewPager = (CustomViewPager) findViewById(R.id.pager);
        mViewPager.setAdapter(mViewPagerAdapter);
        mViewPager.addOnPageChangeListener(pageListener);

        if (getResources().getBoolean(R.bool.isTablet))
            mViewPager.setOffscreenPageLimit(2); // Since two fragments is selected in landscape mode, this is used to smooth things out
/*
        int count = mViewPagerAdapter.getCount();
        Resources mResources = getResources();
        boolean landscape = false;
        if (mResources.getBoolean(R.bool.isTablet) && mResources.getConfiguration().orientation == Configuration.ORIENTATION_LANDSCAPE) {
            landscape = true;
            count -= 1; // There is one less tab when in landscape mode
        }
*/
        try {
            PackageManager mPackageManager = getPackageManager();
            if (mPackageManager != null) {
                BalanduinoActivity.appVersion = mPackageManager.getPackageInfo(getPackageName(), 0).versionName; // Read the app version name
                if (D)
                    Log.d(LOG_TAG, "onCreate() : app version " + appVersion);
            }
        } catch (NameNotFoundException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onStart() {
        super.onStart();
        if (D)
            Log.d(LOG_TAG, "++ ON START ++");

        mPreferences = PreferenceManager.getDefaultSharedPreferences(this); // Create SharedPreferences instance
        String filterCoefficient = mPreferences.getString("filterCoefficient", null); // Read the stored value for filter coefficient
        if (filterCoefficient != null) {
            mSensorFusion.filter_coefficient = Float.parseFloat(filterCoefficient);
            mSensorFusion.tempFilter_coefficient = mSensorFusion.filter_coefficient;
        }
        // Read the previous back to spot value
        backToSpot = mPreferences.getBoolean("backToSpot", true); // Back to spot is true by default
        // Read the previous max angle
        maxAngle = mPreferences.getInt("maxAngle", 8); // Eight is the default value
        // Read the previous max turning value
        maxTurning = mPreferences.getInt("maxTurning", 20); // Twenty is the default value
        String bluetothDeviceAddr = mPreferences.getString("bt_remote_device", null);
        if (bluetothDeviceAddr != null) {
            BluetoothDevice mBluetoothDevice = mBluetoothScan.getBluetoothDevice(bluetothDeviceAddr);
            mBluetoothConnect.connectBluetooth(mBluetoothDevice);
        }
    }

    @Override
    public void onStop() {
        super.onStop();
        if (D)
            Log.d(LOG_TAG, "-- ON STOP --");
        //
        if (mChatService != null) {
            mChatService.stop();
        }
        // Store the value for FILTER_COEFFICIENT and max angle at shutdown
        Editor edit = PreferenceManager.getDefaultSharedPreferences(this).edit();
        edit.putString("filterCoefficient", Float.toString(mSensorFusion.filter_coefficient));
        edit.putBoolean("backToSpot", backToSpot);
        edit.putInt("maxAngle", maxAngle);
        edit.putInt("maxTurning", maxTurning);
        edit.commit();

    }

    @Override
    public void onBackPressed() {
        Upload.close(); // Close serial communication
        if (mChatService != null) {
            new Handler().postDelayed(new Runnable() {
                public void run() {
                    mChatService.stop(); // Stop the Bluetooth chat services if the user exits the app
                }
            }, 1000); // Wait 1 second before closing the connection, this is needed as onPause() will send stop messages before closing
        }
        finish(); // Exits the app
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (D)
            Log.d(LOG_TAG, "--- ON DESTROY ---");
        if (mSensorFusion != null) {
            mSensorFusion.unregisterListeners();
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (D)
            Log.d(LOG_TAG, "- ON PAUSE -");
        // Unregister sensor listeners to prevent the activity from draining the device's battery.
        mSensorFusion.unregisterListeners();
        // Send stop command and stop sending graph data command
        if (mChatService != null) {
            mChatService.write(sendStop + imuStop + statusStop);
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        if (D)
            Log.d(LOG_TAG, "+ ON RESUME +");
        // Restore the sensor listeners when user resumes the application.
        mSensorFusion.initListeners();
    }

    // Check the tab to the left as well in landscape mode
    public static boolean checkTab(int tab) {
        return (currentTabSelected == tab ||
                (context.getResources().getBoolean(R.bool.isTablet) &&
                        context.getResources().getConfiguration().orientation ==
                                Configuration.ORIENTATION_LANDSCAPE && currentTabSelected == tab - 1));
    }


    public static int getRotation() {
        return activity.getWindowManager().getDefaultDisplay().getRotation();
    }

    public static void showToast(String message, int duration) {
        if (duration != Toast.LENGTH_SHORT && duration != Toast.LENGTH_LONG)
            throw new IllegalArgumentException();
        if (mToast != null)
            mToast.cancel(); // Close the toast if it's already open
        mToast = Toast.makeText(context, message, duration);
        mToast.show();
    }

    // TabLayout.OnTabSelectedListener
    @Override
    public void onTabSelected(TabLayout.Tab tab) {
        if (D) {
            Log.e(LOG_TAG, "onTabSelected() :" + tab.getPosition());
        }
        currentTabSelected = tab.getPosition();
        mViewPager.setCurrentItem(currentTabSelected);
        //
        Resources mResources = getResources();
        if (mResources.getBoolean(R.bool.isTablet) &&
                mResources.getConfiguration().orientation == Configuration.ORIENTATION_LANDSCAPE &&
                currentTabSelected == ViewPagerAdapter.INFO_FRAGMENT) { // Check if the last tab is selected in landscape mode
            currentTabSelected -= 1; // If so don't go any further
        }

        if (checkTab(ViewPagerAdapter.GRAPH_FRAGMENT) && mChatService != null) {
            if (mChatService.getState() == BluetoothChatService.STATE_CONNECTED) {
                mChatService.write(getKalman);
                Log.e(LOG_TAG, "onTabSelected() : sent getKalman ");
                if (GraphFragment.mToggleButton != null) {
                    if (GraphFragment.mToggleButton.isChecked())
                        mChatService.write(imuBegin); // Request data
                    else
                        mChatService.write(imuStop); // Stop sending data
                }
            }
        } else if (checkTab(ViewPagerAdapter.INFO_FRAGMENT) && mChatService != null) {
            if (mChatService.getState() == BluetoothChatService.STATE_CONNECTED) {
                mChatService.write(getInfo); // Update info
                Log.e(LOG_TAG, "onTabSelected() : sent getInfo ");
                if (InfoFragment.mToggleButton != null) {
                    if (InfoFragment.mToggleButton.isChecked())
                        mChatService.write(statusBegin); // Request data
                    else
                        mChatService.write(statusStop); // Stop sending data
                }
            }
        } else if (checkTab(ViewPagerAdapter.PID_FRAGMENT) && mChatService != null) {
            if (mChatService.getState() == BluetoothChatService.STATE_CONNECTED) {
                mChatService.write(getPIDValues);
                Log.e(LOG_TAG, "onTabSelected() : sent getPIDValues ");
            }
        }

        if (!checkTab(ViewPagerAdapter.GRAPH_FRAGMENT)) { // Needed when the user rotates the screen
            InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE); // Hide the keyboard
            imm.hideSoftInputFromWindow(getWindow().getDecorView().getApplicationWindowToken(), 0);
        }

    }

    @Override
    public void onTabUnselected(TabLayout.Tab tab) {
        if (D)
            Log.e(LOG_TAG, "onTabUnselected: " + currentTabSelected);

        if ((checkTab(ViewPagerAdapter.IMU_FRAGMENT) ||
                checkTab(ViewPagerAdapter.JOYSTICK_FRAGMENT)) && mChatService != null) { // Send stop command if the user selects another tab
            if (mChatService.getState() == BluetoothChatService.STATE_CONNECTED)
                mChatService.write(sendStop);
        } else if (checkTab(ViewPagerAdapter.GRAPH_FRAGMENT) && mChatService != null) {
            if (mChatService.getState() == BluetoothChatService.STATE_CONNECTED) {
                mChatService.write(imuStop);
                GraphFragment.mToggleButton.setChecked(false);
                GraphFragment.mToggleButton.setText("Start");
            }
        } else if (checkTab(ViewPagerAdapter.INFO_FRAGMENT) && mChatService != null) {
            if (mChatService.getState() == BluetoothChatService.STATE_CONNECTED) {
                mChatService.write(statusStop);
                InfoFragment.mToggleButton.setChecked(false);
            }
        }
        if (checkTab(ViewPagerAdapter.GRAPH_FRAGMENT)) {
            InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE); // Hide the keyboard
            imm.hideSoftInputFromWindow(getWindow().getDecorView().getApplicationWindowToken(), 0);
        }
    }

    @Override
    public void onTabReselected(TabLayout.Tab tab) {

    }

    @Override
    public boolean onPrepareOptionsMenu(Menu menu) {
        if (D)
            Log.d(LOG_TAG, "onPrepareOptionsMenu");
        menuItemConnect = menu.findItem(R.id.menu_connect); // Find item
        if (mBluetoothConnect.isConnected()) {
            menuItemConnect.setIcon(R.drawable.device_access_bluetooth_connected);
        } else {
            menuItemConnect.setIcon(R.drawable.device_access_bluetooth);
        }
        return true;
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        if (D)
            Log.d(LOG_TAG, "onCreateOptionsMenu");
        //getSupportMenuInflater().inflate(R.menu.menu, menu); // Inflate the menu
        getMenuInflater().inflate(R.menu.menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_connect:
                if (mChatService.getState() == BluetoothChatService.STATE_CONNECTED) {
                    mChatService.stop();
                    menuItemConnect.setIcon(R.drawable.device_access_bluetooth);
                    //supportInvalidateOptionsMenu();
                    PIDFragment.updateButton();
                    //
                    SharedPreferences.Editor preferencesEditor = mPreferences.edit();
                    preferencesEditor.remove("bt_remote_device");
                    preferencesEditor.apply();
                    return true;
                }
                Intent intent = new Intent(this, BluetoothDevListActivity.class);
                startActivityForResult(intent, REQUEST_CONNECT_DEVICE);

                //Intent serverIntent = new Intent(this, DeviceListActivity.class);
                //startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
                if (D)
                    Log.i(LOG_TAG, "onOptionsItemSelected() : called DeviceListActivity");
                return true;
            case R.id.menu_settings:
                // Open up the settings dialog
                SettingsDialogFragment dialogFragment = new SettingsDialogFragment();
                dialogFragment.show(getSupportFragmentManager(), "setting dialog");
                return true;
/*            case android.R.id.home:
                Intent browserIntent = new Intent(Intent.ACTION_VIEW, Uri.parse("http://balanduino.net/"));
                startActivity(browserIntent);
                return true;*/
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (D)
            Log.d(LOG_TAG, "onActivityResult() : result code " + resultCode);
        if (requestCode != REQUEST_CONNECT_DEVICE) {
            return;
        }
        if (resultCode == RESULT_OK) {
            BluetoothDevice bluetoothDevice = data.getParcelableExtra(BluetoothDevListActivity.EXTRA_KEY_DEVICE);
            if (bluetoothDevice != null) {
                mBluetoothConnect.connectBluetooth(bluetoothDevice);
                if (D)
                    Log.e(LOG_TAG, "onActivityResult() - connecting");
            }
        } else if (resultCode == RESULT_CANCELED) {
            if (D)
                Log.e(LOG_TAG, "onActivityResult() - canceled");
        }

    }

    @Override
    public void onConnect(BluetoothConnect self) {
        runOnUiThread(new Thread() {
            public void run() {
                Toast.makeText(getApplicationContext(), "Connecting", Toast.LENGTH_SHORT).show();
            }
        });

    }

    @Override
    public void onConnectionSuccess(BluetoothConnect self) {
        if (D)
            Log.e(LOG_TAG, "onConnectionSuccess() :");
        SharedPreferences.Editor preferencesEditor = mPreferences.edit();
        preferencesEditor.putString("bt_remote_device", mBluetoothConnect.getDeviceAddress());
        preferencesEditor.apply();

        runOnUiThread(new Thread() {
            public void run() {
                Toast.makeText(getApplicationContext(), "Connected", Toast.LENGTH_SHORT).show();
            }
        });
        mChatService.connected();
    }

    @Override
    public void onConnectionFail(BluetoothConnect self) {


        runOnUiThread(new Thread() {
            public void run() {
                Toast.makeText(getApplicationContext(),
                        "Connecting fail!",
                        Toast.LENGTH_LONG).show();
            }
        });
    }

    @Override
    public void onDisconnected(BluetoothConnect self) {
        if (mChatService != null) {
            mChatService.stop();
        }
        runOnUiThread(new Thread() {
            public void run() {
                menuItemConnect.setIcon(R.drawable.device_access_bluetooth);
                Toast.makeText(getApplicationContext(),
                        "Connection Lost!",
                        Toast.LENGTH_LONG).show();
                //supportInvalidateOptionsMenu();
                PIDFragment.updateButton();
            }
        });
        if (D)
            Log.e(LOG_TAG, "onDisconnected() :");
    }

    class BluetoothHandler extends Handler {
        private final BalanduinoActivity mBalanduinoActivity;

        BluetoothHandler(BalanduinoActivity mBalanduinoActivity) {
            this.mBalanduinoActivity = mBalanduinoActivity;
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case MESSAGE_STATE_CHANGE:
                    mBalanduinoActivity.supportInvalidateOptionsMenu();  // to show icon for bluetooth connected
                    if (msg.arg1 == BluetoothChatService.STATE_CONNECTED) {
                        BalanduinoActivity.showToast(mBalanduinoActivity.getString(R.string.connected_to) +
                                " " + mBluetoothConnect.getDeviceName(), Toast.LENGTH_SHORT);
                        //mChatService.write(getKalman);
                    }
                    /*
                    switch (msg.arg1) {
                        case BluetoothChatService.STATE_CONNECTED:
                            BalanduinoActivity.showToast(mBalanduinoActivity.getString(R.string.connected_to) +
                                    " " + mBluetoothConnect.getDeviceName(), Toast.LENGTH_SHORT);
                            if (mChatService == null)
                                return;

                            Handler mHandler = new Handler();
                            mHandler.postDelayed(new Runnable() {
                                public void run() {
                                    mChatService.write(getPIDValues + getSettings + getInfo + getKalman);
                                }
                            }, 1000); // Wait 1 second before sending the message

                            if (GraphFragment.mToggleButton != null) {
                                if (GraphFragment.mToggleButton.isChecked() && checkTab(ViewPagerAdapter.GRAPH_FRAGMENT)) {
                                    mHandler.postDelayed(new Runnable() {
                                        public void run() {
                                            mChatService.write(imuBegin); // Request data
                                        }
                                    }, 1000); // Wait 1 second before sending the message
                                } else {
                                    mHandler.postDelayed(new Runnable() {
                                        public void run() {
                                            mChatService.write(imuStop); // Stop sending data
                                        }
                                    }, 1000); // Wait 1 second before sending the message
                                }
                            }
                            if (checkTab(ViewPagerAdapter.INFO_FRAGMENT)) {
                                mHandler.postDelayed(new Runnable() {
                                    public void run() {
                                        mChatService.write(statusBegin); // Request data
                                    }
                                }, 1000); // Wait 1 second before sending the message
                            }

                            break;
                    }
                     */
                    PIDFragment.updateButton();
                    break;
                case MESSAGE_READ:
                    if (newPIDValues) {
                        newPIDValues = false;
                        PIDFragment.updateView();
                    }
                    if (newInfo || newStatus) {
                        newInfo = false;
                        newStatus = false;
                        InfoFragment.updateView();
                    }
                    if (newIMUValues) {
                        newIMUValues = false;
                        GraphFragment.updateIMUValues();
                    }
                    if (newKalmanValues) {
                        newKalmanValues = false;
                        GraphFragment.updateKalmanValues();
                    }
                    if (pairingWithDevice) {
                        pairingWithDevice = false;
                        BalanduinoActivity.showToast("Now enable discovery of your device", Toast.LENGTH_LONG);
                    }
                    break;
            }
        }
    }
}